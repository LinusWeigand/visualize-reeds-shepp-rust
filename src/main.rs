use macroquad::prelude::*;
// Import UI elements (built-in in macroquad 0.4+)
use macroquad::ui::{hash, root_ui, widgets};

// Import the modified library elements
use reeds_shepp_lib::{
    Gear, Path, Pose, Steering, // Basic types
    PATH_FNS,           // Array of path functions
    reflect, timeflip,  // Transformation functions
    path_length,        // Utility
    utils,              // Access utils
    get_all_paths, get_optimal_path, // Need these now
};

// --- Constants ---
const WINDOW_WIDTH: i32 = 1024;
const WINDOW_HEIGHT: i32 = 768;
const CAR_WIDTH: f32 = 30.0;
const CAR_LENGTH: f32 = 50.0;
const PATH_RESOLUTION: f64 = 30.0;
const TURNING_RADIUS: f64 = 1.0;
const DRAW_SCALE: f32 = 50.0;
const HEADLIGHT_SIZE_SCREEN: f32 = 8.0;
const BEAM_LENGTH: f32 = 60.0;
const BEAM_WIDTH: f32 = 40.0;
const TURNING_CIRCLE_OPACITY: f32 = 0.15;

// --- Colors ---
const BG_COLOR: Color = Color::new(0.15, 0.15, 0.18, 1.0);
const START_CAR_COLOR: Color = Color::new(0.7, 0.9, 0.7, 1.0);
const END_CAR_COLOR: Color = Color::new(0.4, 0.5, 0.9, 1.0);
const SELECTED_PATH_COLOR: Color = Color::new(1.0, 0.6, 0.1, 1.0);
const ALL_PATHS_COLOR: Color = Color::new(0.8, 0.8, 0.8, 0.3);
const HEADLIGHT_COLOR: Color = Color::new(1.0, 1.0, 0.7, 1.0);
const BEAM_COLOR: Color = Color::new(1.0, 1.0, 0.5, 0.4);
const TURNING_CIRCLE_COLOR: Color = Color::new(0.8, 0.8, 0.8, TURNING_CIRCLE_OPACITY);

// --- Enums ---
#[derive(PartialEq, Debug, Clone, Copy)]
enum AppState {
    PlacingStart, DefiningStartAngle, PlacingEnd, DefiningEndAngle, DisplayingPaths,
}

#[derive(Clone, Copy, Debug)]
struct InitialDragState { start_pos: Vec2, current_pos: Vec2, }

#[derive(PartialEq, Debug, Clone, Copy)]
enum ModifyDragTarget { StartBody, StartAngle, EndBody, EndAngle, }

#[derive(Debug, Clone, PartialEq)]
enum DisplayMode {
    ShortestPath,
    AllPaths,
    SinglePath(usize), // Index 0-11
}

// --- State Struct ---
struct State {
    app_state: AppState,
    start_pose: Option<Pose>,
    end_pose: Option<Pose>,
    drag_state_initial: Option<InitialDragState>,
    dragging_modify: Option<ModifyDragTarget>,
    // UI & Path State
    display_mode: DisplayMode,
    reflect_path: bool,
    timeflip_path: bool,
    // Drawing Data
    current_path_points: Option<Vec<Vec2>>,
    current_raw_path: Option<Path>,
    all_paths_points: Vec<Vec<Vec2>>,
    // UI State
    combo_box_selected_index: usize, // 0=Shortest, 1=All, 2=Path 1, ...
}

impl State {
    fn new() -> Self {
        State {
            app_state: AppState::PlacingStart,
            start_pose: None, end_pose: None,
            drag_state_initial: None, dragging_modify: None,
            display_mode: DisplayMode::ShortestPath, // Default mode
            reflect_path: false, timeflip_path: false,
            current_path_points: None, current_raw_path: None,
            all_paths_points: Vec::new(),
            combo_box_selected_index: 0, // Matches default mode
        }
    }

    fn reset(&mut self) {
        println!("State reset."); // Print *before* overwriting
        let default_mode = DisplayMode::ShortestPath;
        let default_combo_index = 0;
        *self = State::new();
        self.display_mode = default_mode;
        self.combo_box_selected_index = default_combo_index;
        // No need to print again here
    }

    fn set_display_mode_from_index(&mut self, index: usize) {
        self.combo_box_selected_index = index;
        self.display_mode = match index {
            0 => DisplayMode::ShortestPath,
            1 => DisplayMode::AllPaths,
            i if (2..=13).contains(&i) => DisplayMode::SinglePath(i - 2),
            _ => DisplayMode::ShortestPath, // Fallback
        };
        if !matches!(self.display_mode, DisplayMode::SinglePath(_)) {
            self.reflect_path = false; self.timeflip_path = false;
        }
    }

    fn screen_to_world(&self, screen_pos: Vec2) -> (f64, f64) {
        ( ((screen_pos.x - WINDOW_WIDTH as f32 / 2.0) / DRAW_SCALE) as f64,
          ((WINDOW_HEIGHT as f32 / 2.0 - screen_pos.y) / DRAW_SCALE) as f64 )
    }

    fn world_to_screen_static(world_x: f64, world_y: f64) -> Vec2 {
        vec2( world_x as f32 * DRAW_SCALE + WINDOW_WIDTH as f32 / 2.0,
              WINDOW_HEIGHT as f32 / 2.0 - world_y as f32 * DRAW_SCALE )
    }

    fn calculate_initial_drag_angle(&self) -> Option<f64> {
        if let Some(drag)=&self.drag_state_initial{let d=drag.current_pos-drag.start_pos;if d.length_squared()>5.0*5.0{let a=d.y.atan2(d.x);return Some((-a).to_degrees() as f64);}}None
    }

     fn check_body_hit(&self, world_click_pos: (f64, f64), pose: &Pose) -> bool {
         let angle_rad=pose.theta_degree.to_radians();let cos_a=angle_rad.cos();let sin_a=angle_rad.sin();let dx=world_click_pos.0-pose.x;let dy=world_click_pos.1-pose.y;let local_x=dx*cos_a+dy*sin_a;let local_y=(-dx*sin_a)+dy*cos_a;let h_len=(CAR_LENGTH/DRAW_SCALE/2.0)as f64;let h_wid=(CAR_WIDTH/DRAW_SCALE/2.0)as f64;local_x.abs()<=h_len&&local_y.abs()<=h_wid
     }

    fn get_headlight_world_pos(pose: &Pose) -> (f64, f64) {
        let angle_rad=pose.theta_degree.to_radians();let off=(CAR_LENGTH/DRAW_SCALE/2.0)as f64;(pose.x+off*angle_rad.cos(),pose.y+off*angle_rad.sin())
    }

    fn check_headlight_hit(&self, world_click_pos: (f64, f64), pose: &Pose) -> bool {
        let(hx,hy)=Self::get_headlight_world_pos(pose);let dx=world_click_pos.0-hx;let dy=world_click_pos.1-hy;let d_sq=dx*dx+dy*dy;let r=(HEADLIGHT_SIZE_SCREEN/DRAW_SCALE*1.5)as f64;d_sq<r*r
    }

    // Calculate display data based on mode
    fn calculate_display_data(&mut self) {
        self.current_path_points = None; self.current_raw_path = None;
        self.all_paths_points.clear();
        if let (Some(start), Some(end)) = (self.start_pose.as_ref(), self.end_pose.as_ref()) {
            match self.display_mode {
                DisplayMode::SinglePath(index) => {
                    let relative_pose = utils::change_of_basis(start, end);
                    let mut x=relative_pose.x; let mut y=relative_pose.y; let mut theta_degree=relative_pose.theta_degree;
                    let reflect=self.reflect_path; let timeflip=self.timeflip_path;
                    if reflect { y=-y; theta_degree=-theta_degree; }
                    if timeflip { x=-x; theta_degree = if reflect {theta_degree} else {-theta_degree}; }
                    if let Some(path_fn) = PATH_FNS.get(index) {
                        let mut path = path_fn(x, y, theta_degree);
                        if timeflip { path = reeds_shepp_lib::timeflip(path); } // Use module path
                        if reflect { path = reeds_shepp_lib::reflect(path); } // Use module path
                        if !path.is_empty() {
                            let points = generate_path_points(start, &path, PATH_RESOLUTION);
                            if !points.is_empty() { self.current_path_points=Some(points); self.current_raw_path=Some(path); }
                        }
                    }
                }
                DisplayMode::ShortestPath => {
                    if let Some(path) = get_optimal_path(*start, *end) {
                         if !path.is_empty() {
                            let points = generate_path_points(start, &path, PATH_RESOLUTION);
                            if !points.is_empty() { self.current_path_points=Some(points); self.current_raw_path=Some(path); }
                         }
                    }
                }
                DisplayMode::AllPaths => {
                     let all_raw = get_all_paths(*start, *end);
                     let mut shortest: Option<Path> = None; let mut shortest_len = f64::INFINITY;
                     for path in all_raw {
                          if path.is_empty() { continue; }
                          let points = generate_path_points(start, &path, PATH_RESOLUTION);
                          if !points.is_empty() {
                              self.all_paths_points.push(points);
                              let len = path_length(&path);
                              if len < shortest_len { shortest_len=len; shortest=Some(path); }
                          }
                     }
                     if let Some(s_path) = shortest {
                           let s_points = generate_path_points(start, &s_path, PATH_RESOLUTION);
                           if !s_points.is_empty() { self.current_path_points=Some(s_points); self.current_raw_path=Some(s_path); }
                           else if !self.all_paths_points.is_empty() { self.current_path_points = self.all_paths_points.first().cloned(); }
                     }
                }
            }
        }
    }
} // end impl State


// Generate points for drawing
fn generate_path_points(
    start_pose: &Pose,
    path: &Path,
    resolution: f64,
) -> Vec<Vec2> {
    if path.is_empty() { return Vec::new(); }
    let mut points = Vec::new();
    let mut current_x = start_pose.x; let mut current_y = start_pose.y;
    let mut current_theta_rad = utils::normalize_angle_rad(start_pose.theta_degree.to_radians());
    points.push(State::world_to_screen_static(current_x, current_y));
    for element in path {
        let param = element.param; if param < 1e-10 { continue; }
        let len_res = match element.steering{Steering::Straight=>param,Steering::Left|Steering::Right=>param.abs()*TURNING_RADIUS,};
        let n_steps=((len_res*resolution).ceil().max(1.0))as usize;
        let g_mult=match element.gear{Gear::Forward=>1.0,Gear::Backwards=>-1.0,};
        let mut nx; let mut ny; let mut nt;
        for _i in 1..=n_steps {
             match element.steering {
                Steering::Straight=>{let d=param/n_steps as f64*g_mult; nx=current_x+d*current_theta_rad.cos(); ny=current_y+d*current_theta_rad.sin(); nt=current_theta_rad;}
                Steering::Left=>{let a=param/n_steps as f64*g_mult; nt=utils::normalize_angle_rad(current_theta_rad+a); let dx=TURNING_RADIUS*(nt.sin()-current_theta_rad.sin()); let dy=TURNING_RADIUS*(current_theta_rad.cos()-nt.cos()); nx=current_x+dx; ny=current_y+dy;}
                Steering::Right=>{let a=param/n_steps as f64*g_mult; nt=utils::normalize_angle_rad(current_theta_rad-a); let dx=TURNING_RADIUS*(current_theta_rad.sin()-nt.sin()); let dy=TURNING_RADIUS*(nt.cos()-current_theta_rad.cos()); nx=current_x+dx; ny=current_y+dy;}
            };
             current_x=nx; current_y=ny; current_theta_rad=nt;
             points.push(State::world_to_screen_static(current_x, current_y));
        }
    } points
}

// Draw car, beams, NO turning circles
fn draw_pose_elements(pose: &Pose, body_color: Color) {
    let center_screen = State::world_to_screen_static(pose.x, pose.y);
    let rotation_rad_world = pose.theta_degree.to_radians();
    let rotation_rad_screen = -rotation_rad_world as f32;
    let (headlight_x_world, headlight_y_world) = State::get_headlight_world_pos(pose);
    let headlight_screen = State::world_to_screen_static(headlight_x_world, headlight_y_world);
    let beam_direction = Vec2::from_angle(rotation_rad_screen).normalize();
    let beam_normal = vec2(-beam_direction.y, beam_direction.x);
    let beam_start1 = headlight_screen + beam_normal*(HEADLIGHT_SIZE_SCREEN*0.5);
    let beam_start2 = headlight_screen - beam_normal*(HEADLIGHT_SIZE_SCREEN*0.5);
    let beam_end1 = headlight_screen + beam_direction*BEAM_LENGTH + beam_normal*(BEAM_WIDTH*0.5);
    let beam_end2 = headlight_screen + beam_direction*BEAM_LENGTH - beam_normal*(BEAM_WIDTH*0.5);
    draw_triangle(beam_start1, beam_end1, beam_end2, BEAM_COLOR);
    draw_triangle(beam_start1, beam_end2, beam_start2, BEAM_COLOR);
    draw_rectangle_ex( center_screen.x, center_screen.y, CAR_LENGTH, CAR_WIDTH, DrawRectangleParams { offset:vec2(0.5, 0.5), rotation:rotation_rad_screen, color:body_color, ..Default::default() },);
    draw_circle(headlight_screen.x, headlight_screen.y, HEADLIGHT_SIZE_SCREEN, RED);
    draw_circle(headlight_screen.x, headlight_screen.y, HEADLIGHT_SIZE_SCREEN*0.6, HEADLIGHT_COLOR);
}

// Draw paths based on display mode
fn draw_paths(state: &State) {
    match state.display_mode {
        DisplayMode::SinglePath(_) | DisplayMode::ShortestPath => {
             if let Some(points) = &state.current_path_points { if points.len() > 1 { for i in 0..(points.len() - 1) { draw_line( points[i].x, points[i].y, points[i + 1].x, points[i + 1].y, 3.0, SELECTED_PATH_COLOR, ); } } }
        }
        DisplayMode::AllPaths => {
             for points in &state.all_paths_points { if points.len() > 1 { for i in 0..(points.len() - 1) { draw_line( points[i].x, points[i].y, points[i + 1].x, points[i + 1].y, 1.0, ALL_PATHS_COLOR, ); } } }
             if let Some(points) = &state.current_path_points { if points.len() > 1 { for i in 0..(points.len() - 1) { draw_line( points[i].x, points[i].y, points[i + 1].x, points[i + 1].y, 3.0, SELECTED_PATH_COLOR, ); } } }
        }
    }
}

// Draw turning circles along a specific path
fn draw_path_turning_circles(path: &Path, start_pose: &Pose) {
    if path.is_empty() { return; }
    let mut current_x = start_pose.x; let mut current_y = start_pose.y;
    let mut current_theta_rad = utils::normalize_angle_rad(start_pose.theta_degree.to_radians());
    let turning_radius_screen = TURNING_RADIUS as f32 * DRAW_SCALE;
    for element in path {
        let gear_mult = match element.gear { Gear::Forward => 1.0, Gear::Backwards => -1.0 };
        match element.steering {
            Steering::Straight => { let dist = element.param * gear_mult; current_x += dist * current_theta_rad.cos(); current_y += dist * current_theta_rad.sin(); }
            Steering::Left => { let cx=current_x-TURNING_RADIUS*current_theta_rad.sin(); let cy=current_y+TURNING_RADIUS*current_theta_rad.cos(); let cs=State::world_to_screen_static(cx,cy); draw_circle_lines(cs.x,cs.y,turning_radius_screen,1.0,TURNING_CIRCLE_COLOR,); draw_circle(cs.x,cs.y,2.0,TURNING_CIRCLE_COLOR); let angle=element.param*gear_mult; let next_theta=utils::normalize_angle_rad(current_theta_rad+angle); current_x=cx+TURNING_RADIUS*next_theta.sin(); current_y=cy-TURNING_RADIUS*next_theta.cos(); current_theta_rad=next_theta; }
            Steering::Right => { let cx=current_x+TURNING_RADIUS*current_theta_rad.sin(); let cy=current_y-TURNING_RADIUS*current_theta_rad.cos(); let cs=State::world_to_screen_static(cx,cy); draw_circle_lines(cs.x,cs.y,turning_radius_screen,1.0,TURNING_CIRCLE_COLOR,); draw_circle(cs.x,cs.y,2.0,TURNING_CIRCLE_COLOR); let angle=element.param*gear_mult; let next_theta=utils::normalize_angle_rad(current_theta_rad-angle); current_x=cx-TURNING_RADIUS*next_theta.sin(); current_y=cy+TURNING_RADIUS*next_theta.cos(); current_theta_rad=next_theta; }
        }
    }
}

// Draw UI text instructions and widgets
fn draw_ui(state: &mut State) {
    // Instructions Text / Coords / Pose Info / Path Info / Dragging Text / Angle Def Line
    let text = match state.app_state { AppState::PlacingStart=>"Click START pos",AppState::DefiningStartAngle=>"Drag/release START angle",AppState::PlacingEnd=>"Click END pos",AppState::DefiningEndAngle=>"Drag/release END angle",AppState::DisplayingPaths=>"Drag Body/H'light. Use UI. 'R' Reset.",}; draw_text(text, 20.0, 30.0, 24.0, WHITE);
    let mouse_pos_screen=mouse_position(); let(mouse_x_world,mouse_y_world)=state.screen_to_world(vec2(mouse_pos_screen.0,mouse_pos_screen.1)); let coord_text=format!("World:({:.2},{:.2})",mouse_x_world,mouse_y_world); draw_text(&coord_text,20.0,60.0,20.0,LIGHTGRAY);
    if let Some(p)=state.start_pose{let t=format!("Start:({:.1},{:.1},{:.1}°)",p.x,p.y,p.theta_degree);draw_text(&t,20.0,WINDOW_HEIGHT as f32-60.0,18.0,START_CAR_COLOR);}
    if let Some(p)=state.end_pose{let t=format!("End:  ({:.1},{:.1},{:.1}°)",p.x,p.y,p.theta_degree);draw_text(&t,20.0,WINDOW_HEIGHT as f32-40.0,18.0,END_CAR_COLOR);}
    if let Some(ref p)=state.current_raw_path{let t=match state.display_mode{DisplayMode::SinglePath(idx)=>format!("P{} Len:{:.2}",idx+1,path_length(p)),DisplayMode::ShortestPath|DisplayMode::AllPaths=>format!("Shortest Len:{:.2}",path_length(p)),};draw_text(&t,20.0,WINDOW_HEIGHT as f32-20.0,18.0,SELECTED_PATH_COLOR);}
    let drag_mode_text=match state.dragging_modify{Some(ModifyDragTarget::StartBody)=>"Mov Start",Some(ModifyDragTarget::StartAngle)=>"Rot Start",Some(ModifyDragTarget::EndBody)=>"Mov End",Some(ModifyDragTarget::EndAngle)=>"Rot End",None=>"",}; if!drag_mode_text.is_empty(){draw_text(drag_mode_text,20.0,90.0,20.0,YELLOW);}
    if let Some(drag)=&state.drag_state_initial{if state.app_state==AppState::DefiningStartAngle||state.app_state==AppState::DefiningEndAngle{draw_line(drag.start_pos.x,drag.start_pos.y,drag.current_pos.x,drag.current_pos.y,2.0,YELLOW); if let Some(angle)=state.calculate_initial_drag_angle(){let t=format!("{:.1}°",angle);draw_text(&t,drag.current_pos.x+10.0,drag.current_pos.y,20.0,YELLOW);}}}

    // --- Draw UI Widgets ---
    if state.app_state == AppState::DisplayingPaths {
        // *** FIX: Increase UI Width ***
        let ui_width = 200.0; // Make wider
        let ui_x = WINDOW_WIDTH as f32 - ui_width - 20.0; // Adjust x position
        let ui_y = 20.0; let ui_height = 120.0;

        root_ui().window(hash!(), vec2(ui_x, ui_y), vec2(ui_width, ui_height), |ui| {
            ui.label(None, "Display Mode:");
            let mut mode_labels: Vec<String> = vec![ "Shortest Path".to_string(), "All Paths".to_string(), ];
            mode_labels.extend((1..=12).map(|i| format!("Path {}", i)));
            let mode_labels_str: Vec<&str> = mode_labels.iter().map(|s| s.as_str()).collect();

            // This ComboBox should now hopefully render text correctly with more width
            widgets::ComboBox::new(hash!("display_mode_select"), &mode_labels_str)
                .ui(ui, &mut state.combo_box_selected_index);

            ui.separator();

            let is_single_path_mode = matches!(state.display_mode, DisplayMode::SinglePath(_));
            if is_single_path_mode {
                 ui.checkbox(hash!("reflect_check"), "Reflect", &mut state.reflect_path);
                 ui.checkbox(hash!("timeflip_check"), "Timeflip", &mut state.timeflip_path);
            } else {
                 // Draw disabled text labels
                 ui.label(None, "Reflect (N/A)");
                 ui.label(None, "Timeflip (N/A)");
            }
        });
    }
}

fn window_conf() -> Conf { Conf{window_title:"Reeds-Shepp Path Visualizer".to_owned(),window_width:WINDOW_WIDTH,window_height:WINDOW_HEIGHT,..Default::default()} }

#[macroquad::main(window_conf)]
async fn main() {
    let mut state = State::new();
    loop {
        let mouse_screen=vec2(mouse_position().0,mouse_position().1); let(world_x,world_y)=state.screen_to_world(mouse_screen);
        let old_combo_box_index = state.combo_box_selected_index;
        let old_reflect=state.reflect_path; let old_timeflip=state.timeflip_path;
        let mut needs_recalculation = false;

        // *** FIX: Remove 'continue' from reset logic ***
        if is_key_pressed(KeyCode::R) {
            state.reset();
            // continue; // REMOVED
        }

        // --- Input/State Logic ---
        match state.app_state {
            AppState::PlacingStart=>{if is_mouse_button_pressed(MouseButton::Left){state.start_pose=Some(Pose{x:world_x,y:world_y,theta_degree:0.0});state.drag_state_initial=Some(InitialDragState{start_pos:mouse_screen,current_pos:mouse_screen});state.app_state=AppState::DefiningStartAngle;}}
            AppState::DefiningStartAngle=>{if is_mouse_button_down(MouseButton::Left){if let Some(drag)=&mut state.drag_state_initial{drag.current_pos=mouse_screen;}if let Some(angle)=state.calculate_initial_drag_angle(){if let Some(start)=&mut state.start_pose{start.theta_degree=angle;}}}else if is_mouse_button_released(MouseButton::Left){if let Some(angle)=state.calculate_initial_drag_angle(){if let Some(start)=&mut state.start_pose{start.theta_degree=angle;}}state.drag_state_initial=None;state.app_state=AppState::PlacingEnd;}}
            AppState::PlacingEnd=>{if is_mouse_button_pressed(MouseButton::Left){state.end_pose=Some(Pose{x:world_x,y:world_y,theta_degree:0.0});state.drag_state_initial=Some(InitialDragState{start_pos:mouse_screen,current_pos:mouse_screen});state.app_state=AppState::DefiningEndAngle;}}
            AppState::DefiningEndAngle=>{if is_mouse_button_down(MouseButton::Left){if let Some(drag)=&mut state.drag_state_initial{drag.current_pos=mouse_screen;}if let Some(angle)=state.calculate_initial_drag_angle(){if let Some(end)=&mut state.end_pose{end.theta_degree=angle;}}}else if is_mouse_button_released(MouseButton::Left){if let Some(angle)=state.calculate_initial_drag_angle(){if let Some(end)=&mut state.end_pose{end.theta_degree=angle;}}state.drag_state_initial=None;state.calculate_display_data();state.app_state=AppState::DisplayingPaths;}}
            AppState::DisplayingPaths=>{let ui_rect=Rect::new(WINDOW_WIDTH as f32-200.0-20.0,20.0,200.0,120.0);/*Adjusted width*/let mouse_over_ui=ui_rect.contains(mouse_screen);if is_mouse_button_pressed(MouseButton::Left)&&!mouse_over_ui{let mut tf=false;if let Some(p)=&state.start_pose{if state.check_headlight_hit((world_x,world_y),p){state.dragging_modify=Some(ModifyDragTarget::StartAngle);tf=true;}}if!tf{if let Some(p)=&state.end_pose{if state.check_headlight_hit((world_x,world_y),p){state.dragging_modify=Some(ModifyDragTarget::EndAngle);tf=true;}}}if!tf{if let Some(p)=&state.start_pose{if state.check_body_hit((world_x,world_y),p){state.dragging_modify=Some(ModifyDragTarget::StartBody);tf=true;}}}if!tf{if let Some(p)=&state.end_pose{if state.check_body_hit((world_x,world_y),p){state.dragging_modify=Some(ModifyDragTarget::EndBody);}}}}if let Some(target)=state.dragging_modify{if is_mouse_button_down(MouseButton::Left){match target{ModifyDragTarget::StartBody=>{if let Some(p)=&mut state.start_pose{p.x=world_x;p.y=world_y;needs_recalculation=true;}}ModifyDragTarget::EndBody=>{if let Some(p)=&mut state.end_pose{p.x=world_x;p.y=world_y;needs_recalculation=true;}}ModifyDragTarget::StartAngle=>{if let Some(p)=&mut state.start_pose{let dx=world_x-p.x;let dy=world_y-p.y;if dx.hypot(dy)>1e-6{p.theta_degree=dy.atan2(dx).to_degrees();needs_recalculation=true;}}}ModifyDragTarget::EndAngle=>{if let Some(p)=&mut state.end_pose{let dx=world_x-p.x;let dy=world_y-p.y;if dx.hypot(dy)>1e-6{p.theta_degree=dy.atan2(dx).to_degrees();needs_recalculation=true;}}}}}else if is_mouse_button_released(MouseButton::Left){state.dragging_modify=None;needs_recalculation=true;}}}
        } // end match state.app_state

        // --- Drawing ---
        clear_background(BG_COLOR);

        match state.display_mode { // Draw circles conditionally
            DisplayMode::SinglePath(_) | DisplayMode::ShortestPath => { if let (Some(path), Some(start)) = (&state.current_raw_path, &state.start_pose) { draw_path_turning_circles(path, start); } }
            DisplayMode::AllPaths => {}
        }
        draw_paths(&state); // Draw path lines
        if let Some(ref pose)=state.start_pose{draw_pose_elements(pose,START_CAR_COLOR);}
        if let Some(ref pose)=state.end_pose{draw_pose_elements(pose,END_CAR_COLOR);}
        draw_ui(&mut state); // Draw UI

        // --- Check for UI changes & Update Display Mode ---
        if state.app_state==AppState::DisplayingPaths {
             if state.combo_box_selected_index != old_combo_box_index {
                 state.set_display_mode_from_index(state.combo_box_selected_index);
                 needs_recalculation = true;
             } else if matches!(state.display_mode, DisplayMode::SinglePath(_)) && (state.reflect_path != old_reflect || state.timeflip_path != old_timeflip) {
                 needs_recalculation = true;
             }
        }

        // --- Recalculate if needed ---
        if needs_recalculation { state.calculate_display_data(); }

        next_frame().await
    } // end loop
} // end main
