use std::{thread, sync::{mpsc, Arc}, time::Duration};
use zmq;
use serde_json;
use tokio::sync::Mutex;

pub mod errors {
    #[derive(Debug, Clone)]
    pub enum RskError {
        OtherError(String),
        GameControllerRefusal(String)
    }
    impl<T: RskErrorable> From<T> for RskError {
        fn from(t: T) -> Self {
            t.to_rsk_err()
        }
    }

    pub trait RskErrorable {
        fn to_rsk_err(&self) -> RskError;
    }
    impl<T: ToString> RskErrorable for T {
        fn to_rsk_err(&self) -> RskError {
            RskError::OtherError(self.to_string())
        }
    }

    pub type RskResult<T> = Result<T, RskError>;
}
use errors::*;

fn angle_wrap(angle: f32) -> f32 {
    let mut angle = angle;
    while angle > std::f32::consts::PI {
        angle -= 2.0 * std::f32::consts::PI;
    }
    while angle < -std::f32::consts::PI {
        angle += 2.0 * std::f32::consts::PI;
    }
    angle
}

#[derive(Debug, Clone, Copy)]
pub enum Team {
    Green,
    Blue
}
impl ToString for Team {
    fn to_string(&self) -> String {
        match self {
            Self::Green => "green".to_string(),
            Self::Blue => "blue".to_string()
        }
    }
}
impl Team {
    pub fn other(&self) -> Self {
        match self {
            Self::Green => Self::Blue,
            Self::Blue => Self::Green
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum RobotNumber {
    One,
    Two
}
impl ToString for RobotNumber {
    fn to_string(&self) -> String {
        match self {
            Self::One => "1".to_string(),
            Self::Two => "2".to_string()
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Command {
    Kick(f32),
    Control(AbsPose),
    Teleport(AbsPose),
    Leds(GameDataLedsValue),
    Beep(u16, u16),
    BallTeleport(AbsPose)
}
impl ToString for Command {
    fn to_string(&self) -> String {
        match self {
            Self::Kick(a) => format!(r#"["kick", {a}]"#),
            Self::Control(p) => format!(r#"["control", {}, {}, {}]"#, p.0, p.1, p.get_theta()),
            Self::Teleport(p) => format!(r#"["teleport", {}, {}, {}]"#, p.0, p.1, p.get_theta()),
            Self::Leds(a) => format!(r#"["leds", {}, {}, {}]"#, a.0, a.1, a.2),
            Self::Beep(a, b) => format!(r#"["beep", {a}, {b}]"#),
            Self::BallTeleport(p) => format!(r#"["teleport", {}, {}, {}]"#, p.0, p.1, p.get_theta())
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Destination {
    pub pose: AbsPose,
    pub speed: f32,
    pub rotation_speed: f32
}
impl Destination {
    pub fn new(pose: AbsPose, speed: f32, rotation_speed: f32) -> Self {
        Self {
            pose,
            speed,
            rotation_speed
        }
    }
    pub fn from_pose(pose: AbsPose) -> Self {
        Self::new(pose, 3.0, 10.0)
    }
    pub fn to_control_command(&self, actual_pose:AbsPose) -> Command {
        let mut vector = MoveVector::new(self.pose.0, self.pose.1);

        // Translate the vector to the robot's frame
        vector.0 -= actual_pose.0;
        vector.1 -= actual_pose.1;

        // Rotate the vector to the robot's frame
        let cos = angle_wrap(-actual_pose.2).cos();
        let sin = angle_wrap(-actual_pose.2).sin();

        vector = MoveVector::new(
            vector.0 * cos - vector.1 * sin,
            vector.0 * sin + vector.1 * cos
        );

        // Smooth the vector and do magic
        let distance = (actual_pose - self.pose).norm();
        let mut speed = self.speed;

        let t = distance;

        speed =  (2.0 * t * (1.0 - t) + t.powi(2)) * speed;

        vector = MoveVector::new(
            (vector.0 * speed) / vector.norm(),
            (vector.1 * speed) / vector.norm()
        );

        // Compute the angle
        let speed = self.rotation_speed;

        // Apply speed
        let mut angle = angle_wrap(self.pose.2 - actual_pose.2);
        angle *= speed;

        Command::Control(AbsPose(
            vector.0 + actual_pose.0,
            vector.1 + actual_pose.1,
            angle
        ))
    }
}

pub struct Robot<'a> {
    pub position: Position,
    pub orientation: Orientation,
    pub leds: GameDataLedsValue,
    pub penalized: bool,
    pub penalized_remaining: Option<u32>,
    pub penalized_reason: Option<String>,
    pub preempted: bool,
    pub preemption_reasons: Vec<String>,
    pub number: RobotNumber,
    pub team: Team,
    pub client: &'a Client
}
impl std::fmt::Debug for Robot<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Robot")
            .field("position", &self.position)
            .field("orientation", &self.orientation)
            .field("leds", &self.leds)
            .field("penalized", &self.penalized)
            .field("penalized_remaining", &self.penalized_remaining)
            .field("penalized_reason", &self.penalized_reason)
            .field("preempted", &self.preempted)
            .field("preemption_reasons", &self.preemption_reasons)
            .field("number", &self.number)
            .field("team", &self.team)
            .finish()
    }
}
impl Robot<'_> {
    pub async fn send_command(&self, command: Command) -> RskResult<()> {
        self.client.send_command(self.team, self.number, command).await
    }
    pub fn refresh(&mut self) -> RskResult<()> {
        let robot = self.client.get_robot(self.team, self.number)?;
        self.position = robot.position;
        self.orientation = robot.orientation;
        self.leds = robot.leds;
        self.penalized = robot.penalized;
        self.penalized_remaining = robot.penalized_remaining;
        self.penalized_reason = robot.penalized_reason;
        self.preempted = robot.preempted;
        self.preemption_reasons = robot.preemption_reasons;
        Ok(())
    }
    pub fn get_pose(&mut self) -> RskResult<AbsPose> {
        self.refresh()?;
        Ok(self.get_cache_pose())
    }
    pub fn get_cache_pose(&self) -> AbsPose {
        AbsPose::new(
            self.position.0,
            self.position.1,
            self.orientation.0
        )
    }
    pub async fn get_destination(&self) -> RskResult<Option<Destination>> {
        self.client.get_destination(self.team, self.number).await
    }
    pub async fn is_arrived(&self) -> RskResult<bool> {
        self.client.is_arrived(self.team, self.number).await
    }
    pub async fn goto(&self, destination: Destination) -> RskResult<()> {
        self.client.goto(self.team, self.number, destination).await
    }
    pub async fn stop(&self) -> RskResult<()> {
        self.client.stop(self.team, self.number).await
    }
    pub async fn goto_blocking(&self, destination: Destination) -> RskResult<()> {
        self.client.goto_blocking(self.team, self.number, destination).await
    }
    pub async fn kick(&self, power: f32) -> RskResult<()> {
        self.client.send_command(self.team, self.number, Command::Kick(power)).await
    }
}

pub mod game_data_structs {
    use serde::{Serialize, Deserialize};

    use super::{Team, Client, errors::RskResult, angle_wrap};
    /* Sample game data:
    {
        "markers": {
          "green1": { "position": [-0.5, 0.5], "orientation": 0.0 },
          "green2": { "position": [-0.5, -0.5], "orientation": 0.0 },
          "blue1": { "position": [0.5, 0.5], "orientation": 0.0 },
          "blue2": { "position": [0.5, -0.5], "orientation": 0.0 }
        },
        "ball": [0.0, 0.0],
        "referee": {
          "game_is_running": false,
          "game_paused": true,
          "halftime_is_running": false,
          "timer": 0,
          "game_state_msg": "Game is ready to start",
          "teams": {
            "green": {
              "name": "",
              "score": 0,
              "x_positive": true,
              "robots": {
                "1": {
                  "penalized": false,
                  "penalized_remaining": null,
                  "penalized_reason": null,
                  "preempted": false,
                  "preemption_reasons": []
                },
                "2": {
                  "penalized": false,
                  "penalized_remaining": null,
                  "penalized_reason": null,
                  "preempted": false,
                  "preemption_reasons": []
                }
              }
            },
            "blue": {
              "name": "",
              "score": 0,
              "x_positive": false,
              "robots": {
                "1": {
                  "penalized": false,
                  "penalized_remaining": null,
                  "penalized_reason": null,
                  "preempted": false,
                  "preemption_reasons": []
                },
                "2": {
                  "penalized": false,
                  "penalized_remaining": null,
                  "penalized_reason": null,
                  "preempted": false,
                  "preemption_reasons": []
                }
              }
            }
          },
          "referee_history_sliced": [0, -9263, "neutral", "Sideline crossed"]
        },
        "leds": {
          "green1": [0, 50, 0],
          "green2": [0, 50, 0],
          "blue1": [0, 0, 50],
          "blue2": [0, 0, 50]
        },
        "simulated": true
      }
    */
    pub trait Pose: Serialize + for<'a> Deserialize<'a> + std::fmt::Debug + Clone + Copy + PartialEq + std::ops::Sub {
        fn new(x: f32, y: f32, theta: f32) -> Self;
        fn get_x(&self) -> f32;
        fn get_y(&self) -> f32;
        fn get_theta(&self) -> f32;
        fn set_x(&mut self, x: f32);
        fn set_y(&mut self, y: f32);
        fn set_theta(&mut self, theta: f32);
    }

    #[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
    pub struct AbsPose(
        pub f32,
        pub f32,
        pub f32
    );
    impl Pose for AbsPose {
        fn new(x: f32, y: f32, theta: f32) -> Self {AbsPose(x, y, angle_wrap(theta))}
        fn get_x(&self) -> f32 {self.0}
        fn get_y(&self) -> f32 {self.1}
        fn get_theta(&self) -> f32 {self.2}
        fn set_x(&mut self, x: f32) {self.0 = x;}
        fn set_y(&mut self, y: f32) {self.1 = y;}
        fn set_theta(&mut self, theta: f32) {
            self.2 = angle_wrap(theta);
        }
    }
    impl std::ops::Sub for AbsPose {
        type Output = MoveVector;
        fn sub(self, other: AbsPose) -> MoveVector {
            MoveVector (
                other.get_x() - self.get_x(),
                other.get_y() - self.get_y()
            )
        }
    }
    impl std::ops::Add<MoveVector> for AbsPose {
        type Output = AbsPose;
        fn add(self, other: MoveVector) -> AbsPose {
            AbsPose (
                self.get_x() + other.0,
                self.get_y() + other.1,
                self.get_theta()
            )
        }
    }
    impl AbsPose {
        pub fn get_rel(&self, team:Team, client:&Client) -> RskResult<RelPose> {
            let gd = client.get_game_data()?;
            let x_positive = match team {
                Team::Blue => gd.referee.teams.blue.x_positive,
                Team::Green => gd.referee.teams.green.x_positive
            };
            let mut rel_pose = RelPose::new(
                self.get_x(),
                self.get_y(),
                self.get_theta()
            );
            if x_positive {
                rel_pose.set_x(rel_pose.get_x() * -1.0);
                rel_pose.set_theta(rel_pose.get_theta() + std::f32::consts::PI);
            }
            Ok(rel_pose)
        }
    }

    #[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
    pub struct RelPose (
        pub f32,
        pub f32,
        pub f32
    );
    impl Pose for RelPose {
        fn new(x: f32, y: f32, theta: f32) -> Self {
            RelPose(x, y, angle_wrap(theta))
        }
        fn get_x(&self) -> f32 {self.0}
        fn get_y(&self) -> f32 {self.1}
        fn get_theta(&self) -> f32 {self.2}
        fn set_x(&mut self, x: f32) {self.0 = x;}
        fn set_y(&mut self, y: f32) {self.1 = y;}
        fn set_theta(&mut self, theta: f32) {
            self.2 = angle_wrap(theta);
        }
    }
    impl std::ops::Sub for RelPose {
        type Output = MoveVector;
        fn sub(self, other: RelPose) -> MoveVector {
            MoveVector (
                other.get_x() - self.get_x(),
                other.get_y() - self.get_y()
            )
        }
    }
    impl RelPose {
        pub fn get_abs(&self, team:Team, client:&Client) -> RskResult<AbsPose> {
            let gd = client.get_game_data()?;
            let x_positive = match team {
                Team::Blue => gd.referee.teams.blue.x_positive,
                Team::Green => gd.referee.teams.green.x_positive
            };
            let mut abs_pose = AbsPose::new(
                self.get_x(),
                self.get_y(),
                self.get_theta()
            );
            if x_positive {
                abs_pose.set_x(abs_pose.get_x() * -1.0);
                abs_pose.set_theta(abs_pose.get_theta() + std::f32::consts::PI);
            }
            Ok(abs_pose)
        }
    }

    #[derive(Serialize, Deserialize, Debug, Clone, Copy)]
    pub struct Position(
        pub f32,
        pub f32
    );
    impl std::ops::Sub for Position {
        type Output = MoveVector;
        fn sub(self, other: Position) -> MoveVector {
            MoveVector (
                other.0 - self.0,
                other.1 - self.1
            )
        }
    }

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct Orientation(
        pub f32
    );
    impl Orientation {
        pub fn new(theta: f32) -> Self {
            Orientation(angle_wrap(theta))
        }
    }

    #[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
    pub struct MoveVector (
        pub f32,
        pub f32
    );
    impl std::ops::Mul<f32> for MoveVector {
        type Output = MoveVector;
        fn mul(self, other: f32) -> MoveVector {
            MoveVector (
                self.0 * other,
                self.1 * other
            )
        }
    }
    impl std::ops::Div<f32> for MoveVector {
        type Output = MoveVector;
        fn div(self, other: f32) -> MoveVector {
            MoveVector (
                self.0 / other,
                self.1 / other
            )
        }
    }
    impl MoveVector {
        pub fn new(x: f32, y: f32) -> Self {
            MoveVector(x, y)
        }
        pub fn norm(&self) -> f32 {
            (self.0.powi(2) + self.1.powi(2)).sqrt()
        }
    }

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct GameDataMarkersRobot {
        pub position: Position,
        pub orientation: Orientation
    }

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct GameDataMarkers {
        pub green1: GameDataMarkersRobot,
        pub green2: GameDataMarkersRobot,
        pub blue1: GameDataMarkersRobot,
        pub blue2: GameDataMarkersRobot
    }

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct GameDataRefereeTeamRobot {
        pub penalized: bool,
        pub penalized_remaining: Option<u32>,
        pub penalized_reason: Option<String>,
        pub preempted: bool,
        pub preemption_reasons: Vec<String>
    }

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct GameDataRefereeTeamRobots {
        pub one: GameDataRefereeTeamRobot,
        pub two: GameDataRefereeTeamRobot
    }

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct GameDataRefereeTeam {
        pub name: String,
        pub score: u8,
        pub x_positive: bool,
        pub robots: GameDataRefereeTeamRobots
    }

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct GameDataRefereeTeams {
        pub green: GameDataRefereeTeam,
        pub blue: GameDataRefereeTeam
    }

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct GameDataReferee {
        pub game_is_running: bool,
        pub game_paused: bool,
        pub halftime_is_running: bool,
        pub timer: i32,
        pub game_state_msg: String,
        pub teams: GameDataRefereeTeams,
        pub referee_history_sliced: Vec<Vec<serde_json::Value>>
    }

    #[derive(Serialize, Deserialize, Debug, Clone, Copy)]
    pub struct GameDataLedsValue(
        pub u8,
        pub u8,
        pub u8
    );

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct GameDataLeds {
        pub green1: GameDataLedsValue,
        pub green2: GameDataLedsValue,
        pub blue1: GameDataLedsValue,
        pub blue2: GameDataLedsValue
    }

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct GameData {
        pub markers: GameDataMarkers,
        pub ball: Position,
        pub referee: GameDataReferee,
        pub leds: GameDataLeds,
        pub simulated: bool
    }
}
use game_data_structs::*;

#[derive(Debug, Clone, Copy)]
pub struct Destinations {
    pub blue1: Option<Destination>,
    pub blue2: Option<Destination>,
    pub green1: Option<Destination>,
    pub green2: Option<Destination>
}

#[derive(Clone)]
pub struct Client {
    pub key: String,
    pub host: String,
    pub team: Team,
    cmd_sender: mpsc::SyncSender<(Team, RobotNumber, Command)>,
    game_data: Arc<std::sync::Mutex<Option<GameData>>>,
    pub destinations: Arc<Mutex<Destinations>>
}
impl Client {
    pub fn new(key:String, host:String, team:Team) -> RskResult<Client> {
        let (
            cmd_sender,
            rx
        ) = mpsc::sync_channel::<(Team, RobotNumber, Command)>(10);
        let key_clone = key.clone();
        let host_clone = host.clone();
        thread::spawn(move || {
            loop {
                match Client::req_process(&key_clone, &host_clone, &rx) {
                    Ok(_) => (),
                    Err(e) => println!("Client::req_process() :\n{}", match e {
                        RskError::OtherError(s) => s,
                        RskError::GameControllerRefusal(s) => s
                    })
                }
            }
        });
        
        let game_data = Arc::new(std::sync::Mutex::new(None));
        let game_data_clone = game_data.clone();
        let host_clone = host.clone();
        thread::spawn(move || {
            loop {
                match Client::sub_process(&host_clone, game_data_clone.clone()) {
                    Ok(_) => (),
                    Err(e) => println!("Client::sub_process() :\n{}", match e {
                        RskError::OtherError(s) => s,
                        RskError::GameControllerRefusal(s) => s
                    })
                }
            }
        });
        let client = Client {
            key,
            host,
            team,
            cmd_sender,
            game_data,
            destinations: Arc::new(Mutex::new(Destinations {
                blue1: None,
                blue2: None,
                green1: None,
                green2: None
            }))
        };

        let client_clone = client.clone();
        tokio::spawn(async move {
            loop {
                if let Err(e) = Self::goto_process(client_clone.clone(), team, RobotNumber::One).await {
                    println!("Error in goto process 1: {:?}", e);
                }
            }
        });

        let client_clone = client.clone();
        tokio::spawn(async move {
            loop {
                if let Err(e) = Self::goto_process(client_clone.clone(), team, RobotNumber::Two).await {
                    println!("Error in goto process 2: {:?}", e);
                }
            }
        });
        
        // Wait for the first game data
        let mut gd = None;
        while let None = gd {
            gd = (*client.game_data.lock()?).clone();
        }
        
        Ok(client)
    }
    pub async fn send_command(&self, team:Team, number:RobotNumber, command:Command) -> RskResult<()> {
        self.cmd_sender.send((team, number, command))?;
        Ok(())
    }
    pub fn req_process(key: &str, host: &str, rx: &mpsc::Receiver<(Team, RobotNumber, Command)>) -> RskResult<()> {
        let ctx = zmq::Context::new();
        let req = ctx.socket(zmq::REQ)?;
        req.connect(&("tcp://".to_string() + host + ":7558"))?;

        loop {
            let (
                team,
                number,
                command
            ) = rx.recv()?;

            let team_str = if let Command::BallTeleport(_) = command {
                "ball".to_string()
            } else {
                team.to_string()
            };

            req.send(
                zmq::Message::from(
                    &format!(r#"["{0}", "{1}", {2}, {3}]"#, key, team_str, number.to_string(), command.to_string())
                ),
                0
            )?;

            let response = match req.recv_string(0)? {
                Err(e) => return Err(format!("Client.send_command() : the received message is not a string. raw data : {:?}", e).to_rsk_err()),
                Ok(v) => v
            };

            let response: Vec<serde_json::Value> = serde_json::from_str(&response)?;

            match response.get(0).unwrap_or(&serde_json::Value::Null) {
                serde_json::Value::Bool(b) => {
                    if !b {
                        let message = match response.get(1).unwrap_or(&serde_json::Value::Null) {
                            serde_json::Value::String(s) => s,
                            _ => return Err("Client.send_command() : the second index of the received array is not a string".to_rsk_err())
                        };
        
                        return Err(("Client.send_command() : the server returned an error : ".to_string() + message).to_rsk_err());
                    }
                },
                serde_json::Value::Number(_) => {
                    println!("Client.reqprocess() : {}", response.get(1)
                        .unwrap_or(
                            &serde_json::Value::String("Client.send_command() : the second index of the received array is not a string".to_string())
                        )
                    );
                },
                _ => return Err("Client.send_command() : the first index of the received array is not a boolean".to_rsk_err())
            };
        }
    }
    pub fn sub_process(host: &str, game_data: Arc<std::sync::Mutex<Option<GameData>>>) -> RskResult<()> {
        let ctx = zmq::Context::new();
        let sub = ctx.socket(zmq::SUB)?;
        sub.connect(&("tcp://".to_string() + host + ":7557"))?;
        sub.set_subscribe(b"")?;
        
        loop {
            let mut response = match sub.recv_string(0)? {
                Err(e) => return Err(format!("the received message is not a string.\n\nraw data : {:?}", e).to_rsk_err()),
                Ok(v) => v
            };

            response = response.replace(r#""1":"#, r#""one":"#);
            response = response.replace(r#""2":"#, r#""two":"#);
    
            let received_game_data: GameData = match serde_json::from_str(&response) {
                Err(e) => return Err(format!("the received message is not a valid json.\n\ninitial error : {:?}\n\nraw game data :\n{}", e, response).to_rsk_err()),
                Ok(v) => v
            };

            let mut game_data = game_data.lock()?;
            *game_data = Some(received_game_data);
        }
    }
    pub async fn goto_process(client: Client, team:Team, number:RobotNumber) -> RskResult<()> {
        loop {
            let destination = client.get_destination(team, number).await?;
            let actual_pose = client.get_robot(team, number)?.get_pose()?;
            if let Some(destination) = destination {
                if let Err(e) = client.cmd_sender.send((team, number, destination.to_control_command(actual_pose))) {
                    println!("Client.goto_process() : error while sending command : {:?}", e);
                }
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
    }
    pub async fn get_destinations(&self) -> RskResult<Destinations> {
        let destinations = (*self.destinations.lock().await).clone();
        Ok(destinations)
    }
    pub fn get_game_data(&self) -> RskResult<GameData> {
        let gd = (*self.game_data.lock()?).clone();
        match gd {
            Some(gd) => Ok(gd),
            None => Err("Client.get_game_data() : no game data available".to_rsk_err())
        }
    }
    pub fn get_robot(&self, team:Team, number:RobotNumber) -> RskResult<Robot> {
        let gd = self.get_game_data()?;
        let markers_robot = match team {
            Team::Blue => {
                match number {
                    RobotNumber::One => gd.markers.blue1,
                    RobotNumber::Two => gd.markers.blue2
                }
            },
            Team::Green => {
                match number {
                    RobotNumber::One => gd.markers.green1,
                    RobotNumber::Two => gd.markers.green2
                }
            }
        };
        let referee_robot = match team {
            Team::Blue => {
                match number {
                    RobotNumber::One => gd.referee.teams.blue.robots.one,
                    RobotNumber::Two => gd.referee.teams.blue.robots.two
                }
            },
            Team::Green => {
                match number {
                    RobotNumber::One => gd.referee.teams.green.robots.one,
                    RobotNumber::Two => gd.referee.teams.green.robots.two
                }
            }
        };
        let leds = match team {
            Team::Blue => {
                match number {
                    RobotNumber::One => gd.leds.blue1,
                    RobotNumber::Two => gd.leds.blue2
                }
            },
            Team::Green => {
                match number {
                    RobotNumber::One => gd.leds.green1,
                    RobotNumber::Two => gd.leds.green2
                }
            }
        };
        Ok(Robot {
            team,
            number,
            position: markers_robot.position,
            orientation: markers_robot.orientation,
            leds,
            penalized: referee_robot.penalized,
            penalized_remaining: referee_robot.penalized_remaining,
            penalized_reason: referee_robot.penalized_reason,
            preempted: referee_robot.preempted,
            preemption_reasons: referee_robot.preemption_reasons,
            client: &self
        })
    }
    pub async fn is_arrived(&self, team:Team, number:RobotNumber) -> RskResult<bool> {
        let robot = self.get_robot(team, number)?;
        let destination = self.get_destination(team, number).await?;
        if let Some(destination) = destination {
            return Ok((robot.get_cache_pose() - destination.pose).norm() < 0.03 && (robot.get_cache_pose().get_theta() - destination.pose.get_theta()).abs() < 0.05);
        }
        Ok(true)
    }
    pub async fn get_destination(&self, team:Team, number:RobotNumber) -> RskResult<Option<Destination>> {
        let destinations = self.destinations.lock().await;
        let destination = match team {
            Team::Blue => {
                match number {
                    RobotNumber::One => destinations.blue1,
                    RobotNumber::Two => destinations.blue2
                }
            },
            Team::Green => {
                match number {
                    RobotNumber::One => destinations.green1,
                    RobotNumber::Two => destinations.green2
                }
            }
        };
        Ok(destination)
    }
    pub async fn set_destination(&self, team:Team, number:RobotNumber, destination:Option<Destination>) -> RskResult<()> {
        let mut destinations = self.destinations.lock().await;
        match team {
            Team::Blue => {
                match number {
                    RobotNumber::One => destinations.blue1 = destination,
                    RobotNumber::Two => destinations.blue2 = destination
                }
            },
            Team::Green => {
                match number {
                    RobotNumber::One => destinations.green1 = destination,
                    RobotNumber::Two => destinations.green2 = destination
                }
            }
        };
        Ok(())
    }
    pub async fn goto(&self, team:Team, number:RobotNumber, destination:Destination) -> RskResult<()> {
        self.set_destination(team, number, Some(destination)).await?;
        Ok(())
    }
    pub async fn stop(&self, team:Team, number:RobotNumber) -> RskResult<()> {
        self.set_destination(team, number, None).await?;
        Ok(())
    }
    pub async fn goto_blocking(&self, team:Team, number:RobotNumber, destination:Destination) -> RskResult<()> {
        loop {
            self.goto(team, number, destination).await?;
            if self.is_arrived(team, number).await? {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        Ok(())
    }
    pub fn get_ball(&self) -> RskResult<Position> {
        Ok(self.get_game_data()?.ball)
    }
    pub async fn get_allies(&self) -> RskResult<(Robot, Robot)> {
        Ok((self.get_robot(self.team, RobotNumber::One)?, self.get_robot(self.team, RobotNumber::Two)?))
    }
    pub async fn get_enemies(&self) -> RskResult<(Robot, Robot)> {
        Ok((self.get_robot(self.team.other(), RobotNumber::One)?, self.get_robot(self.team.other(), RobotNumber::Two)?))
    }
}
