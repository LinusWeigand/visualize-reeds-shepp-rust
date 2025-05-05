use macroquad::prelude::*;
// Import UI elements (built-in in macroquad 0.4+)
use macroquad::ui::{hash, root_ui, widgets}; // Removed unused Ui

// Import the modified library elements
use reeds_shepp_lib::{
    Gear,
    PATH_FNS, // Array of path functions
    Path,
    Pose,
    Steering,    // Basic types (Removed unused PathElement)
    path_length, // Utility
    reflect,
    timeflip, // Transformation functions
    utils,    // Access utils like change_of_basis if needed directly
};
// Removed unused PI

const WINDOW_WIDTH: i32 = 1024;
const WINDOW_HEIGHT: i32 = 768;
const CAR_WIDTH: f32 = 30.0;
const CAR_LENGTH: f32 = 50.0;
const PATH_RESOLUTION: f64 = 30.0; // Increased resolution for smoother curves
const TURNING_RADIUS: f64 = 1.0; // Corresponds to radius=1 in the library functions
const DRAW_SCALE: f32 = 50.0;
// const BODY_HIT_RADIUS_WORLD: f64 = (CAR_LENGTH / DRAW_SCALE / 2.0) as f64; // Using rect check now
const HEADLIGHT_SIZE_SCREEN: f32 = 8.0;
// const HEADLIGHT_HIT_RADIUS_WORLD: f64 = (HEADLIGHT_SIZE_SCREEN / DRAW_SCALE * 1.5) as f64; // Using screen check now
const BEAM_LENGTH: f32 = 60.0;
const BEAM_WIDTH: f32 = 40.0;
const TURNING_CIRCLE_OPACITY: f32 = 0.15; // Slightly more transparent

// Colors
const BG_COLOR: Color = Color::new(0.15, 0.15, 0.18, 1.0);
const START_CAR_COLOR: Color = Color::new(0.7, 0.9, 0.7, 1.0); // Light green
const END_CAR_COLOR: Color = Color::new(0.4, 0.5, 0.9, 1.0); // Blue
const SELECTED_PATH_COLOR: Color = Color::new(1.0, 0.6, 0.1, 1.0); // Orange for selected path
const HEADLIGHT_COLOR: Color = Color::new(1.0, 1.0, 0.7, 1.0); // Light yellow
const BEAM_COLOR: Color = Color::new(1.0, 1.0, 0.5, 0.4); // Slightly less opaque beam
const TURNING_CIRCLE_COLOR: Color = Color::new(0.8, 0.8, 0.8, TURNING_CIRCLE_OPACITY); // Light gray

#[derive(PartialEq, Debug, Clone, Copy)]
enum AppState {
    PlacingStart,
    DefiningStartAngle,
    PlacingEnd,
    DefiningEndAngle,
    DisplayingPaths,
}

#[derive(Clone, Copy, Debug)]
struct InitialDragState {
    start_pos: Vec2,
    current_pos: Vec2,
}

#[derive(PartialEq, Debug, Clone, Copy)]
enum ModifyDragTarget {
    StartBody,
    StartAngle,
    EndBody,
    EndAngle,
}

struct State {
    app_state: AppState,
    start_pose: Option<Pose>,
    end_pose: Option<Pose>,
    drag_state_initial: Option<InitialDragState>,
    dragging_modify: Option<ModifyDragTarget>,
    // UI State
    selected_path_index: usize, // Index 0-11 for path type
    reflect_path: bool,
    timeflip_path: bool,
    // Store calculated path points for drawing
    current_path_points: Option<Vec<Vec2>>,
    // Store the raw path for potential info display
    current_raw_path: Option<Path>,
    // Cache labels for combobox
    path_labels_str: Vec<&'static str>, // Need static lifetime for combo box options
}

// Function to create static string slices (leaking memory, but simple for this example)
fn create_static_labels() -> Vec<&'static str> {
    let mut labels = Vec::new();
    for i in 1..=12 {
        let s = Box::leak(format!("Path {}", i).into_boxed_str());
        // Coerce to &'static str
        labels.push(s as &'static str);
    }
    labels
}

impl State {
    fn new() -> Self {
        let path_labels_str = create_static_labels();
        State {
            app_state: AppState::PlacingStart,
            start_pose: None,
            end_pose: None,
            drag_state_initial: None,
            dragging_modify: None,
            selected_path_index: 0,
            reflect_path: false,
            timeflip_path: false,
            current_path_points: None,
            current_raw_path: None,
            path_labels_str,
        }
    }

    fn reset(&mut self) {
        let path_labels_str = self.path_labels_str.clone(); // Keep labels
        *self = State::new();
        self.path_labels_str = path_labels_str; // Restore labels
        println!("State reset.");
    }

    fn screen_to_world(&self, screen_pos: Vec2) -> (f64, f64) {
        (
            ((screen_pos.x - WINDOW_WIDTH as f32 / 2.0) / DRAW_SCALE) as f64,
            ((WINDOW_HEIGHT as f32 / 2.0 - screen_pos.y) / DRAW_SCALE) as f64,
        )
    }

    fn world_to_screen_static(world_x: f64, world_y: f64) -> Vec2 {
        vec2(
            world_x as f32 * DRAW_SCALE + WINDOW_WIDTH as f32 / 2.0,
            WINDOW_HEIGHT as f32 / 2.0 - world_y as f32 * DRAW_SCALE,
        )
    }

    fn calculate_initial_drag_angle(&self) -> Option<f64> {
        if let Some(drag) = &self.drag_state_initial {
            let delta = drag.current_pos - drag.start_pos;
            if delta.length_squared() > 5.0 * 5.0 {
                let angle_rad_screen = delta.y.atan2(delta.x);
                let angle_rad_world = -angle_rad_screen;
                return Some(angle_rad_world.to_degrees() as f64);
            }
        }
        None
    }

    fn check_body_hit(&self, world_click_pos: (f64, f64), pose: &Pose) -> bool {
        let angle_rad = pose.theta_degree.to_radians();
        let cos_a = angle_rad.cos();
        let sin_a = angle_rad.sin();
        let dx = world_click_pos.0 - pose.x;
        let dy = world_click_pos.1 - pose.y;
        let local_x = dx * cos_a + dy * sin_a;
        let local_y = -dx * sin_a + dy * cos_a;
        let half_len_world = (CAR_LENGTH / DRAW_SCALE / 2.0) as f64;
        let half_wid_world = (CAR_WIDTH / DRAW_SCALE / 2.0) as f64;
        local_x.abs() <= half_len_world && local_y.abs() <= half_wid_world
    }

    fn get_headlight_world_pos(pose: &Pose) -> (f64, f64) {
        let angle_rad = pose.theta_degree.to_radians();
        let head_offset = (CAR_LENGTH / DRAW_SCALE / 2.0) as f64;
        (
            pose.x + head_offset * angle_rad.cos(),
            pose.y + head_offset * angle_rad.sin(),
        )
    }

    fn check_headlight_hit(&self, world_click_pos: (f64, f64), pose: &Pose) -> bool {
        let (headlight_x, headlight_y) = Self::get_headlight_world_pos(pose);
        let dx = world_click_pos.0 - headlight_x;
        let dy = world_click_pos.1 - headlight_y;
        let dist_sq = dx * dx + dy * dy;
        let hit_radius_world = (HEADLIGHT_SIZE_SCREEN / DRAW_SCALE * 1.5) as f64;
        dist_sq < hit_radius_world * hit_radius_world
    }

    // Calculate the selected path based on UI state
    fn calculate_selected_path(&mut self) {
        self.current_path_points = None;
        self.current_raw_path = None;
        if let (Some(start), Some(end)) = (self.start_pose, self.end_pose) {
            let relative_pose = utils::change_of_basis(&start, &end);
            let mut x = relative_pose.x;
            let mut y = relative_pose.y;
            let mut theta_degree = relative_pose.theta_degree;
            let needs_timeflip_calc = self.timeflip_path;
            let needs_reflect_calc = self.reflect_path;

            if needs_reflect_calc {
                y = -y;
                theta_degree = -theta_degree;
            }
            if needs_timeflip_calc {
                x = -x;
                theta_degree = if needs_reflect_calc {
                    theta_degree
                } else {
                    -theta_degree
                };
            }

            let path_fn = PATH_FNS[self.selected_path_index];
            let mut calculated_path = path_fn(x, y, theta_degree);

            if needs_timeflip_calc {
                calculated_path = timeflip(calculated_path);
            }
            if needs_reflect_calc {
                calculated_path = reflect(calculated_path);
            }

            if !calculated_path.is_empty() {
                let points = generate_path_points(&start, &end, &calculated_path, PATH_RESOLUTION);
                if !points.is_empty() {
                    self.current_path_points = Some(points);
                    self.current_raw_path = Some(calculated_path);
                    return;
                } // else { println!("Warning: Path {} generated no points...", self.selected_path_index + 1); }
            } // else { println!("Warning: Path {} calculation failed...", self.selected_path_index + 1); }
        }
    }
}

// Generate points for drawing
fn generate_path_points(
    start_pose: &Pose,
    _end_pose: &Pose,
    path: &Path,
    resolution: f64,
) -> Vec<Vec2> {
    if path.is_empty() {
        return Vec::new();
    }

    let mut points = Vec::new();
    let mut current_x = start_pose.x;
    let mut current_y = start_pose.y;
    let mut current_theta_rad = utils::normalize_angle_rad(start_pose.theta_degree.to_radians());

    points.push(State::world_to_screen_static(current_x, current_y));

    for element in path {
        let param = element.param;
        if param < 1e-10 {
            continue;
        }
        let length_for_res = match element.steering {
            Steering::Straight => param,
            Steering::Left | Steering::Right => param.abs() * TURNING_RADIUS,
        };
        let num_steps = ((length_for_res * resolution).ceil().max(1.0)) as usize;
        let gear_mult = match element.gear {
            Gear::Forward => 1.0,
            Gear::Backwards => -1.0,
        };

        let mut next_x;
        let mut next_y;
        let mut next_theta;

        for _i in 1..=num_steps {
            match element.steering {
                Steering::Straight => {
                    let dist_step = param / num_steps as f64 * gear_mult;
                    next_x = current_x + dist_step * current_theta_rad.cos();
                    next_y = current_y + dist_step * current_theta_rad.sin();
                    next_theta = current_theta_rad;
                }
                Steering::Left => {
                    let angle_step = param / num_steps as f64 * gear_mult;
                    next_theta = utils::normalize_angle_rad(current_theta_rad + angle_step);
                    let dx = TURNING_RADIUS * (next_theta.sin() - current_theta_rad.sin());
                    let dy = TURNING_RADIUS * (current_theta_rad.cos() - next_theta.cos());
                    next_x = current_x + dx;
                    next_y = current_y + dy;
                }
                Steering::Right => {
                    let angle_step = param / num_steps as f64 * gear_mult;
                    next_theta = utils::normalize_angle_rad(current_theta_rad - angle_step);
                    let dx = TURNING_RADIUS * (current_theta_rad.sin() - next_theta.sin());
                    let dy = TURNING_RADIUS * (next_theta.cos() - current_theta_rad.cos());
                    next_x = current_x + dx;
                    next_y = current_y + dy;
                }
            };
            current_x = next_x;
            current_y = next_y;
            current_theta_rad = next_theta;
            points.push(State::world_to_screen_static(current_x, current_y));
        }
    }
    points
}

// Draw car, beams, turning circles
fn draw_pose_elements(pose: &Pose, body_color: Color) {
    let center_screen = State::world_to_screen_static(pose.x, pose.y);
    let rotation_rad_world = pose.theta_degree.to_radians();
    let rotation_rad_screen = -rotation_rad_world as f32;

    let left_turn_center_world_x = pose.x - TURNING_RADIUS * rotation_rad_world.sin();
    let left_turn_center_world_y = pose.y + TURNING_RADIUS * rotation_rad_world.cos();
    let right_turn_center_world_x = pose.x + TURNING_RADIUS * rotation_rad_world.sin();
    let right_turn_center_world_y = pose.y - TURNING_RADIUS * rotation_rad_world.cos();
    let left_turn_center_screen =
        State::world_to_screen_static(left_turn_center_world_x, left_turn_center_world_y);
    let right_turn_center_screen =
        State::world_to_screen_static(right_turn_center_world_x, right_turn_center_world_y);
    let turning_radius_screen = TURNING_RADIUS as f32 * DRAW_SCALE;
    draw_circle_lines(
        left_turn_center_screen.x,
        left_turn_center_screen.y,
        turning_radius_screen,
        1.0,
        TURNING_CIRCLE_COLOR,
    );
    draw_circle(
        left_turn_center_screen.x,
        left_turn_center_screen.y,
        2.0,
        TURNING_CIRCLE_COLOR,
    );
    draw_circle_lines(
        right_turn_center_screen.x,
        right_turn_center_screen.y,
        turning_radius_screen,
        1.0,
        TURNING_CIRCLE_COLOR,
    );
    draw_circle(
        right_turn_center_screen.x,
        right_turn_center_screen.y,
        2.0,
        TURNING_CIRCLE_COLOR,
    );

    let (headlight_x_world, headlight_y_world) = State::get_headlight_world_pos(pose);
    let headlight_screen = State::world_to_screen_static(headlight_x_world, headlight_y_world);
    let beam_direction = Vec2::from_angle(rotation_rad_screen).normalize();
    let beam_normal = vec2(-beam_direction.y, beam_direction.x);
    let beam_start1 = headlight_screen + beam_normal * (HEADLIGHT_SIZE_SCREEN * 0.5);
    let beam_start2 = headlight_screen - beam_normal * (HEADLIGHT_SIZE_SCREEN * 0.5);
    let beam_end1 =
        headlight_screen + beam_direction * BEAM_LENGTH + beam_normal * (BEAM_WIDTH * 0.5);
    let beam_end2 =
        headlight_screen + beam_direction * BEAM_LENGTH - beam_normal * (BEAM_WIDTH * 0.5);
    draw_triangle(beam_start1, beam_end1, beam_end2, BEAM_COLOR);
    draw_triangle(beam_start1, beam_end2, beam_start2, BEAM_COLOR);

    draw_rectangle_ex(
        center_screen.x,
        center_screen.y,
        CAR_LENGTH,
        CAR_WIDTH,
        DrawRectangleParams {
            offset: vec2(0.5, 0.5),
            rotation: rotation_rad_screen,
            color: body_color,
            ..Default::default()
        },
    );

    draw_circle(
        headlight_screen.x,
        headlight_screen.y,
        HEADLIGHT_SIZE_SCREEN,
        RED,
    );
    draw_circle(
        headlight_screen.x,
        headlight_screen.y,
        HEADLIGHT_SIZE_SCREEN * 0.6,
        HEADLIGHT_COLOR,
    );
}

// Draw the calculated selected path
fn draw_paths(state: &State) {
    if let Some(points) = &state.current_path_points {
        if points.len() > 1 {
            for i in 0..(points.len() - 1) {
                draw_line(
                    points[i].x,
                    points[i].y,
                    points[i + 1].x,
                    points[i + 1].y,
                    3.0,
                    SELECTED_PATH_COLOR,
                );
            }
        }
    }
}

// Draw UI text instructions and widgets
fn draw_ui(state: &mut State) {
    let text = match state.app_state {
        /* ... */ AppState::PlacingStart => "Click to place START position",
        AppState::DefiningStartAngle => "Drag/release START angle",
        AppState::PlacingEnd => "Click to place END position",
        AppState::DefiningEndAngle => "Drag/release END angle",
        AppState::DisplayingPaths => "Drag Body/Headlight. Use UI. 'R' Reset.",
    };
    draw_text(text, 20.0, 30.0, 24.0, WHITE);
    let mouse_pos_screen = mouse_position();
    let (mouse_x_world, mouse_y_world) =
        state.screen_to_world(vec2(mouse_pos_screen.0, mouse_pos_screen.1));
    let coord_text = format!("World: ({:.2}, {:.2})", mouse_x_world, mouse_y_world);
    draw_text(&coord_text, 20.0, 60.0, 20.0, LIGHTGRAY);
    if let Some(p) = state.start_pose {
        let t = format!("Start: ({:.1}, {:.1}, {:.1}°)", p.x, p.y, p.theta_degree);
        draw_text(&t, 20.0, WINDOW_HEIGHT as f32 - 60.0, 18.0, START_CAR_COLOR);
    }
    if let Some(p) = state.end_pose {
        let t = format!("End:   ({:.1}, {:.1}, {:.1}°)", p.x, p.y, p.theta_degree);
        draw_text(&t, 20.0, WINDOW_HEIGHT as f32 - 40.0, 18.0, END_CAR_COLOR);
    }
    if let Some(ref p) = state.current_raw_path {
        let t = format!("Path Len: {:.2}", path_length(p));
        draw_text(
            &t,
            20.0,
            WINDOW_HEIGHT as f32 - 20.0,
            18.0,
            SELECTED_PATH_COLOR,
        );
    }
    let drag_mode_text = match state.dragging_modify {
        Some(ModifyDragTarget::StartBody) => "Moving Start",
        Some(ModifyDragTarget::StartAngle) => "Rot Start",
        Some(ModifyDragTarget::EndBody) => "Moving End",
        Some(ModifyDragTarget::EndAngle) => "Rot End",
        None => "",
    };
    if !drag_mode_text.is_empty() {
        draw_text(drag_mode_text, 20.0, 90.0, 20.0, YELLOW);
    }
    if let Some(drag) = &state.drag_state_initial {
        if state.app_state == AppState::DefiningStartAngle
            || state.app_state == AppState::DefiningEndAngle
        {
            draw_line(
                drag.start_pos.x,
                drag.start_pos.y,
                drag.current_pos.x,
                drag.current_pos.y,
                2.0,
                YELLOW,
            );
            if let Some(angle) = state.calculate_initial_drag_angle() {
                let t = format!("{:.1}°", angle);
                draw_text(
                    &t,
                    drag.current_pos.x + 10.0,
                    drag.current_pos.y,
                    20.0,
                    YELLOW,
                );
            }
        }
    }

    // --- Draw UI Widgets ---
    if state.app_state == AppState::DisplayingPaths {
        let ui_width = 150.0;
        let ui_x = WINDOW_WIDTH as f32 - ui_width - 20.0;
        let ui_y = 20.0;
        let ui_height = 110.0;
        root_ui().window(hash!(), vec2(ui_x, ui_y), vec2(ui_width, ui_height), |ui| {
            ui.label(None, "Select Path:");
            // Pass options directly to new(), pass &mut index to ui()
            widgets::ComboBox::new(hash!("path_select"), &state.path_labels_str)
                .ui(ui, &mut state.selected_path_index); // Pass mutable reference HERE

            ui.separator();
            ui.checkbox(hash!("reflect_check"), "Reflect", &mut state.reflect_path);
            ui.checkbox(
                hash!("timeflip_check"),
                "Timeflip",
                &mut state.timeflip_path,
            );
        });
    }
}

fn window_conf() -> Conf {
    Conf {
        window_title: "Reeds-Shepp Path Visualizer".to_owned(),
        window_width: WINDOW_WIDTH,
        window_height: WINDOW_HEIGHT,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut state = State::new();
    loop {
        let mouse_screen = vec2(mouse_position().0, mouse_position().1);
        let (world_x, world_y) = state.screen_to_world(mouse_screen);
        let old_selected_index = state.selected_path_index;
        let old_reflect = state.reflect_path;
        let old_timeflip = state.timeflip_path;
        let mut needs_recalculation = false;
        if is_key_pressed(KeyCode::R) {
            state.reset();
            continue;
        }

        match state.app_state {
            AppState::PlacingStart => {
                if is_mouse_button_pressed(MouseButton::Left) {
                    state.start_pose = Some(Pose {
                        x: world_x,
                        y: world_y,
                        theta_degree: 0.0,
                    });
                    state.drag_state_initial = Some(InitialDragState {
                        start_pos: mouse_screen,
                        current_pos: mouse_screen,
                    });
                    state.app_state = AppState::DefiningStartAngle;
                }
            }
            AppState::DefiningStartAngle => {
                if is_mouse_button_down(MouseButton::Left) {
                    if let Some(drag) = &mut state.drag_state_initial {
                        drag.current_pos = mouse_screen;
                    }
                    if let Some(angle) = state.calculate_initial_drag_angle() {
                        if let Some(start) = &mut state.start_pose {
                            start.theta_degree = angle;
                        }
                    }
                } else if is_mouse_button_released(MouseButton::Left) {
                    if let Some(angle) = state.calculate_initial_drag_angle() {
                        if let Some(start) = &mut state.start_pose {
                            start.theta_degree = angle;
                        }
                    }
                    state.drag_state_initial = None;
                    state.app_state = AppState::PlacingEnd;
                }
            }
            AppState::PlacingEnd => {
                if is_mouse_button_pressed(MouseButton::Left) {
                    state.end_pose = Some(Pose {
                        x: world_x,
                        y: world_y,
                        theta_degree: 0.0,
                    });
                    state.drag_state_initial = Some(InitialDragState {
                        start_pos: mouse_screen,
                        current_pos: mouse_screen,
                    });
                    state.app_state = AppState::DefiningEndAngle;
                }
            }
            AppState::DefiningEndAngle => {
                if is_mouse_button_down(MouseButton::Left) {
                    if let Some(drag) = &mut state.drag_state_initial {
                        drag.current_pos = mouse_screen;
                    }
                    if let Some(angle) = state.calculate_initial_drag_angle() {
                        if let Some(end) = &mut state.end_pose {
                            end.theta_degree = angle;
                        }
                    }
                } else if is_mouse_button_released(MouseButton::Left) {
                    if let Some(angle) = state.calculate_initial_drag_angle() {
                        if let Some(end) = &mut state.end_pose {
                            end.theta_degree = angle;
                        }
                    }
                    state.drag_state_initial = None;
                    state.calculate_selected_path();
                    state.app_state = AppState::DisplayingPaths;
                }
            }
            AppState::DisplayingPaths => {
                let ui_rect = Rect::new(WINDOW_WIDTH as f32 - 150.0 - 20.0, 20.0, 150.0, 110.0);
                let mouse_over_ui = ui_rect.contains(mouse_screen);
                if is_mouse_button_pressed(MouseButton::Left) && !mouse_over_ui {
                    let mut target_found = false;
                    if let Some(ref pose) = state.start_pose {
                        if state.check_headlight_hit((world_x, world_y), pose) {
                            state.dragging_modify = Some(ModifyDragTarget::StartAngle);
                            target_found = true;
                        }
                    }
                    if !target_found {
                        if let Some(ref pose) = state.end_pose {
                            if state.check_headlight_hit((world_x, world_y), pose) {
                                state.dragging_modify = Some(ModifyDragTarget::EndAngle);
                                target_found = true;
                            }
                        }
                    }
                    if !target_found {
                        if let Some(ref pose) = state.start_pose {
                            if state.check_body_hit((world_x, world_y), pose) {
                                state.dragging_modify = Some(ModifyDragTarget::StartBody);
                                target_found = true;
                            }
                        }
                    }
                    if !target_found {
                        if let Some(ref pose) = state.end_pose {
                            if state.check_body_hit((world_x, world_y), pose) {
                                state.dragging_modify = Some(ModifyDragTarget::EndBody);
                            }
                        }
                    }
                }
                if let Some(target) = state.dragging_modify {
                    if is_mouse_button_down(MouseButton::Left) {
                        match target {
                            ModifyDragTarget::StartBody => {
                                if let Some(pose) = &mut state.start_pose {
                                    pose.x = world_x;
                                    pose.y = world_y;
                                    needs_recalculation = true;
                                }
                            }
                            ModifyDragTarget::EndBody => {
                                if let Some(pose) = &mut state.end_pose {
                                    pose.x = world_x;
                                    pose.y = world_y;
                                    needs_recalculation = true;
                                }
                            }
                            ModifyDragTarget::StartAngle => {
                                if let Some(pose) = &mut state.start_pose {
                                    let dx = world_x - pose.x;
                                    let dy = world_y - pose.y;
                                    if dx.hypot(dy) > 1e-6 {
                                        pose.theta_degree = dy.atan2(dx).to_degrees();
                                        needs_recalculation = true;
                                    }
                                }
                            }
                            ModifyDragTarget::EndAngle => {
                                if let Some(pose) = &mut state.end_pose {
                                    let dx = world_x - pose.x;
                                    let dy = world_y - pose.y;
                                    if dx.hypot(dy) > 1e-6 {
                                        pose.theta_degree = dy.atan2(dx).to_degrees();
                                        needs_recalculation = true;
                                    }
                                }
                            }
                        }
                    } else if is_mouse_button_released(MouseButton::Left) {
                        state.dragging_modify = None;
                        needs_recalculation = true;
                    }
                }
            }
        }

        clear_background(BG_COLOR);
        draw_paths(&state);
        if let Some(ref pose) = state.start_pose {
            draw_pose_elements(pose, START_CAR_COLOR);
        }
        if let Some(ref pose) = state.end_pose {
            draw_pose_elements(pose, END_CAR_COLOR);
        }
        draw_ui(&mut state); // Draw UI potentially modifies state via &mut state passed in

        if state.app_state == AppState::DisplayingPaths {
            if state.selected_path_index != old_selected_index
                || state.reflect_path != old_reflect
                || state.timeflip_path != old_timeflip
            {
                needs_recalculation = true;
            }
        }
        if needs_recalculation {
            state.calculate_selected_path();
        }

        next_frame().await
    }
}
