use macroquad::prelude::*;
use reeds_shepp_lib::{Gear, Path, PathElement, Pose, Steering, get_all_paths, get_optimal_path}; // Use actual library
use std::f64::consts::PI;

const WINDOW_WIDTH: i32 = 1024;
const WINDOW_HEIGHT: i32 = 768;
const CAR_WIDTH: f32 = 30.0;
const CAR_LENGTH: f32 = 50.0;
const PATH_RESOLUTION: f64 = 20.0; // Number of points per unit distance/radian
const TURNING_RADIUS: f64 = 1.0; // Corresponds to the implicit radius in the calculations
const DRAW_SCALE: f32 = 50.0; // Scale world units for drawing
const BODY_HIT_RADIUS_WORLD: f64 = (CAR_LENGTH / DRAW_SCALE / 2.0) as f64; // Hit radius for body
const HEADLIGHT_SIZE_SCREEN: f32 = 8.0;
const HEADLIGHT_HIT_RADIUS_WORLD: f64 = (HEADLIGHT_SIZE_SCREEN / DRAW_SCALE * 1.5) as f64; // Generous headlight hit radius

// Application states for initial placement and display/modification
#[derive(PartialEq, Debug, Clone, Copy)]
enum AppState {
    PlacingStart,
    DefiningStartAngle,
    PlacingEnd,
    DefiningEndAngle,
    DisplayingPaths, // Also used for modification
}

// Store drag state for initial angle definition
#[derive(Clone, Copy, Debug)]
struct InitialDragState {
    start_pos: Vec2,   // Screen coordinates
    current_pos: Vec2, // Screen coordinates
}

// Track what is being dragged during modification
#[derive(PartialEq, Debug, Clone, Copy)]
enum ModifyDragTarget {
    StartBody,
    StartAngle, // Dragging the headlight
    EndBody,
    EndAngle, // Dragging the headlight
}

// Holds the application's current state
struct State {
    app_state: AppState,
    start_pose: Option<Pose>,
    end_pose: Option<Pose>,
    drag_state_initial: Option<InitialDragState>, // For setting initial angle
    dragging_modify: Option<ModifyDragTarget>,    // For modifying pose after placement
    all_paths_points: Vec<Vec<Vec2>>,
    optimal_path_points: Option<Vec<Vec2>>,
}

impl State {
    fn new() -> Self {
        State {
            app_state: AppState::PlacingStart,
            start_pose: None,
            end_pose: None,
            drag_state_initial: None,
            dragging_modify: None,
            all_paths_points: Vec::new(),
            optimal_path_points: None,
        }
    }

    fn reset(&mut self) {
        *self = State::new();
        println!("State reset.");
    }

    // Converts screen coordinates to world coordinates
    fn screen_to_world(&self, screen_pos: Vec2) -> (f64, f64) {
        (
            ((screen_pos.x - WINDOW_WIDTH as f32 / 2.0) / DRAW_SCALE) as f64,
            ((WINDOW_HEIGHT as f32 / 2.0 - screen_pos.y) / DRAW_SCALE) as f64,
        )
    }

    // Converts world coordinates to screen coordinates
    fn world_to_screen_static(world_x: f64, world_y: f64) -> Vec2 {
        vec2(
            world_x as f32 * DRAW_SCALE + WINDOW_WIDTH as f32 / 2.0,
            WINDOW_HEIGHT as f32 / 2.0 - world_y as f32 * DRAW_SCALE,
        )
    }

    // Calculates the angle from the initial drag operation (screen coords)
    fn calculate_initial_drag_angle(&self) -> Option<f64> {
        if let Some(drag) = &self.drag_state_initial {
            // Check if drag distance is significant enough
            if (drag.current_pos - drag.start_pos).length_squared() > 10.0 * 10.0 {
                // Increased threshold
                // Angle based on screen coordinates drag direction
                let angle_rad_screen = (drag.current_pos.y - drag.start_pos.y)
                    .atan2(drag.current_pos.x - drag.start_pos.x);
                // Convert screen angle (Y down) to world angle (Y up, 0=East)
                return Some((-angle_rad_screen).to_degrees().into());
            }
        }
        None
    }

    // --- Hit detection for modification phase ---

    // Check if a world point hits the car body (approximated as a circle)
    fn check_body_hit(&self, world_click_pos: (f64, f64), pose: &Pose) -> bool {
        let dx = world_click_pos.0 - pose.x;
        let dy = world_click_pos.1 - pose.y;
        let dist_sq = dx * dx + dy * dy;
        dist_sq < BODY_HIT_RADIUS_WORLD * BODY_HIT_RADIUS_WORLD
    }

    // Get the world position of the headlight
    fn get_headlight_world_pos(pose: &Pose) -> (f64, f64) {
        let angle_rad = pose.theta_degree.to_radians();
        let head_offset = (CAR_LENGTH / DRAW_SCALE / 2.0) as f64; // World units
        (
            pose.x + head_offset * angle_rad.cos(),
            pose.y + head_offset * angle_rad.sin(),
        )
    }

    // Check if a world point hits the headlight (approximated as a circle)
    fn check_headlight_hit(&self, world_click_pos: (f64, f64), pose: &Pose) -> bool {
        let (headlight_x, headlight_y) = Self::get_headlight_world_pos(pose);
        let dx = world_click_pos.0 - headlight_x;
        let dy = world_click_pos.1 - headlight_y;
        let dist_sq = dx * dx + dy * dy;
        dist_sq < HEADLIGHT_HIT_RADIUS_WORLD * HEADLIGHT_HIT_RADIUS_WORLD
    }

    // --- Path Calculation ---

    // Triggers path calculation and prepares drawing data
    fn calculate_and_prepare_paths(&mut self) {
        if let (Some(start), Some(end)) = (self.start_pose, self.end_pose) {
            // Use copied poses
            // println!("Calculating paths..."); // Reduce console spam

            // Pass copies to the path functions as Pose might not be Clone
            let all_paths = get_all_paths(
                Pose {
                    x: start.x,
                    y: start.y,
                    theta_degree: start.theta_degree,
                },
                Pose {
                    x: end.x,
                    y: end.y,
                    theta_degree: end.theta_degree,
                },
            );
            let optimal_path_opt = get_optimal_path(
                Pose {
                    x: start.x,
                    y: start.y,
                    theta_degree: start.theta_degree,
                },
                Pose {
                    x: end.x,
                    y: end.y,
                    theta_degree: end.theta_degree,
                },
            );

            self.all_paths_points.clear();
            for path in &all_paths {
                let points = generate_path_points(&start, &end, path, PATH_RESOLUTION);
                if !points.is_empty() {
                    self.all_paths_points.push(points);
                }
            }

            self.optimal_path_points = None;
            if let Some(optimal_path) = optimal_path_opt {
                let optimal_points =
                    generate_path_points(&start, &end, &optimal_path, PATH_RESOLUTION);
                if !optimal_points.is_empty() {
                    self.optimal_path_points = Some(optimal_points);
                } else {
                    println!("Warning: Optimal path found but generated no points.");
                }
            }
        } else {
            // If start or end is None, clear existing paths
            self.all_paths_points.clear();
            self.optimal_path_points = None;
        }
    }
}

// Helper to calculate path length
fn path_length(path: &Path) -> f64 {
    path.iter().map(|e| e.param.abs()).sum()
}

// --- Path Interpolation ---
// (Same as before, generates screen points for drawing a path)
fn generate_path_points(
    start_pose: &Pose,
    end_pose: &Pose, // Still useful for debugging
    path: &Path,
    resolution: f64,
) -> Vec<Vec2> {
    let mut points = Vec::new();
    let mut current_x = start_pose.x;
    let mut current_y = start_pose.y;
    let mut current_theta_rad = start_pose.theta_degree.to_radians();

    points.push(State::world_to_screen_static(current_x, current_y));

    for element in path {
        let param = element.param;
        let num_steps = ((param.abs() * resolution).ceil().max(1.0)) as usize;
        // Ensure step_size calculation avoids division by zero if num_steps is somehow zero
        let step_size = if num_steps > 0 {
            param / num_steps as f64
        } else {
            0.0
        };

        let gear_mult = match element.gear {
            Gear::Forward => 1.0,
            Gear::Backwards => -1.0,
        };

        for _ in 0..num_steps {
            let (dx, dy, dtheta) = match element.steering {
                Steering::Straight => (
                    step_size * gear_mult * current_theta_rad.cos(),
                    step_size * gear_mult * current_theta_rad.sin(),
                    0.0,
                ),
                // Using simplified circle segment formulas for small steps
                Steering::Left => {
                    let angle_step = step_size * gear_mult / TURNING_RADIUS; // step_size is arc length here
                    (
                        TURNING_RADIUS * (current_theta_rad + angle_step).sin()
                            - TURNING_RADIUS * current_theta_rad.sin(),
                        TURNING_RADIUS * current_theta_rad.cos()
                            - TURNING_RADIUS * (current_theta_rad + angle_step).cos(),
                        angle_step,
                    )
                }
                Steering::Right => {
                    let angle_step = step_size * gear_mult / TURNING_RADIUS; // step_size is arc length here
                    (
                        TURNING_RADIUS * current_theta_rad.sin()
                            - TURNING_RADIUS * (current_theta_rad - angle_step).sin(),
                        TURNING_RADIUS * (current_theta_rad - angle_step).cos()
                            - TURNING_RADIUS * current_theta_rad.cos(),
                        -angle_step,
                    )
                }
            };

            current_x += dx;
            current_y += dy;
            current_theta_rad += dtheta;

            points.push(State::world_to_screen_static(current_x, current_y));
        }
    }
    // Optional Debug print (can be uncommented)
    /*
    let final_theta_deg = current_theta_rad.to_degrees().rem_euclid(360.0);
    let target_theta_deg = end_pose.theta_degree.rem_euclid(360.0);
    let dist_err = ((current_x - end_pose.x).powi(2) + (current_y - end_pose.y).powi(2)).sqrt();
    let angle_diff_raw = final_theta_deg - target_theta_deg;
    let angle_err = (angle_diff_raw + 180.0).rem_euclid(360.0) - 180.0;
    if dist_err > 0.1 || angle_err.abs() > 5.0 { // Print only significant errors
        println!("--- Interpolation Discrepancy ---");
        println!(
            "Interpolation End: x={:.3}, y={:.3}, th={:.2}°",
            current_x, current_y, final_theta_deg
        );
        println!(
            "Target End Pose:   x={:.3}, y={:.3}, th={:.2}°",
            end_pose.x, end_pose.y, target_theta_deg
        );
        println!(
            "End Pose Error: dist={:.4}, angle={:.2}°",
            dist_err, angle_err
        );
         println!("Path: {:?}", path); // Print path details on error
        println!("--------------------------------");
    }
    */
    points
}

// --- Drawing Functions ---

// Draw car body and headlight
fn draw_pose_elements(pose: &Pose, body_color: Color) {
    let center_screen = State::world_to_screen_static(pose.x, pose.y);
    let rotation_rad_world = pose.theta_degree.to_radians();
    let rotation_rad_screen = -rotation_rad_world as f32; // For macroquad drawing

    // Draw body
    draw_rectangle_ex(
        center_screen.x,
        center_screen.y,
        CAR_LENGTH,
        CAR_WIDTH,
        DrawRectangleParams {
            offset: vec2(0.5, 0.5), // Center the rectangle origin
            rotation: rotation_rad_screen,
            color: body_color,
            ..Default::default()
        },
    );

    // Calculate headlight position in world and screen coords
    let (headlight_x_world, headlight_y_world) = State::get_headlight_world_pos(pose);
    let headlight_screen = State::world_to_screen_static(headlight_x_world, headlight_y_world);

    // Draw headlight (white circle)
    draw_circle(
        headlight_screen.x,
        headlight_screen.y,
        HEADLIGHT_SIZE_SCREEN,
        WHITE,
    );
    // Optionally add a smaller inner circle for effect
    draw_circle(
        headlight_screen.x,
        headlight_screen.y,
        HEADLIGHT_SIZE_SCREEN * 0.6,
        YELLOW,
    );
}

// Draw the calculated paths
fn draw_paths(state: &State) {
    // Draw all paths (thin, semi-transparent blue)
    for path_points in &state.all_paths_points {
        if path_points.len() > 1 {
            for i in 0..(path_points.len() - 1) {
                draw_line(
                    path_points[i].x,
                    path_points[i].y,
                    path_points[i + 1].x,
                    path_points[i + 1].y,
                    1.0,
                    Color::new(0.5, 0.5, 1.0, 0.4),
                );
            }
        }
    }

    // Draw optimal path (thicker, opaque yellow) on top
    if let Some(optimal_points) = &state.optimal_path_points {
        if optimal_points.len() > 1 {
            for i in 0..(optimal_points.len() - 1) {
                draw_line(
                    optimal_points[i].x,
                    optimal_points[i].y,
                    optimal_points[i + 1].x,
                    optimal_points[i + 1].y,
                    3.0,
                    Color::new(1.0, 1.0, 0.0, 1.0), // Yellow
                );
            }
        }
    }
}

// Draw UI text instructions based on AppState
fn draw_ui(state: &State) {
    let text = match state.app_state {
        AppState::PlacingStart => "Click to place START position",
        AppState::DefiningStartAngle => "Drag and release to set START angle",
        AppState::PlacingEnd => "Click to place END position",
        AppState::DefiningEndAngle => "Drag and release to set END angle",
        AppState::DisplayingPaths => "Drag Body (Move) or Headlight (Rotate). Press 'R' to Reset.",
    };
    draw_text(text, 20.0, 30.0, 24.0, WHITE);

    let mouse_pos_screen = mouse_position();
    let (mouse_x_world, mouse_y_world) =
        state.screen_to_world(vec2(mouse_pos_screen.0, mouse_pos_screen.1));
    let coord_text = format!("World Coords: ({:.2}, {:.2})", mouse_x_world, mouse_y_world);
    draw_text(&coord_text, 20.0, 60.0, 20.0, LIGHTGRAY);

    // Indicate modification drag mode if active
    let drag_mode_text = match state.dragging_modify {
        Some(ModifyDragTarget::StartBody) => "Moving Start",
        Some(ModifyDragTarget::StartAngle) => "Rotating Start",
        Some(ModifyDragTarget::EndBody) => "Moving End",
        Some(ModifyDragTarget::EndAngle) => "Rotating End",
        None => "",
    };
    if !drag_mode_text.is_empty() {
        draw_text(drag_mode_text, 20.0, 90.0, 20.0, YELLOW);
    }

    // Draw the initial angle definition line if active
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
        }
    }
}

// --- Main Application Logic ---

fn window_conf() -> Conf {
    Conf {
        window_title: "Reeds-Shepp Path Visualizer - Place & Modify".to_owned(),
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

        // Global reset key
        if is_key_pressed(KeyCode::R) {
            state.reset();
        }

        // --- Input Handling based on AppState ---
        match state.app_state {
            AppState::PlacingStart => {
                if is_mouse_button_pressed(MouseButton::Left) {
                    state.start_pose = Some(Pose {
                        x: world_x,
                        y: world_y,
                        theta_degree: 0.0, // Default angle
                    });
                    state.drag_state_initial = Some(InitialDragState {
                        start_pos: mouse_screen,
                        current_pos: mouse_screen,
                    });
                    state.app_state = AppState::DefiningStartAngle;
                    println!("Start pos set. Define angle.");
                }
            }
            AppState::DefiningStartAngle => {
                if is_mouse_button_down(MouseButton::Left) {
                    if let Some(drag) = &mut state.drag_state_initial {
                        drag.current_pos = mouse_screen;
                        if let Some(angle_deg) = state.calculate_initial_drag_angle() {
                            if let Some(start) = &mut state.start_pose {
                                start.theta_degree = angle_deg;
                            }
                        }
                    }
                } else if is_mouse_button_released(MouseButton::Left) {
                    if let Some(start) = &state.start_pose {
                        // Use final angle
                        println!("Start angle set to: {:.1}°", start.theta_degree);
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
                        theta_degree: 0.0, // Default angle
                    });
                    state.drag_state_initial = Some(InitialDragState {
                        start_pos: mouse_screen,
                        current_pos: mouse_screen,
                    });
                    state.app_state = AppState::DefiningEndAngle;
                    println!("End pos set. Define angle.");
                }
            }
            AppState::DefiningEndAngle => {
                if is_mouse_button_down(MouseButton::Left) {
                    if let Some(drag) = &mut state.drag_state_initial {
                        drag.current_pos = mouse_screen;
                        if let Some(angle_deg) = state.calculate_initial_drag_angle() {
                            if let Some(end) = &mut state.end_pose {
                                end.theta_degree = angle_deg;
                            }
                        }
                    }
                } else if is_mouse_button_released(MouseButton::Left) {
                    if let Some(end) = &state.end_pose {
                        // Use final angle
                        println!("End angle set to: {:.1}°", end.theta_degree);
                    }
                    state.drag_state_initial = None;
                    // Crucially, calculate paths *after* end angle is set
                    state.calculate_and_prepare_paths();
                    state.app_state = AppState::DisplayingPaths;
                    println!("Poses defined. Displaying paths. Drag to modify.");
                }
            }
            AppState::DisplayingPaths => {
                // --- Modification Drag Logic ---
                if is_mouse_button_pressed(MouseButton::Left) {
                    // Prioritize headlight hits over body hits
                    let mut target_found = false;
                    if let Some(ref pose) = state.start_pose {
                        if state.check_headlight_hit((world_x, world_y), pose) {
                            state.dragging_modify = Some(ModifyDragTarget::StartAngle);
                            target_found = true;
                            println!("Dragging Start Angle");
                        }
                    }
                    if !target_found {
                        if let Some(ref pose) = state.end_pose {
                            if state.check_headlight_hit((world_x, world_y), pose) {
                                state.dragging_modify = Some(ModifyDragTarget::EndAngle);
                                target_found = true;
                                println!("Dragging End Angle");
                            }
                        }
                    }
                    // If no headlight hit, check for body hits
                    if !target_found {
                        if let Some(ref pose) = state.start_pose {
                            if state.check_body_hit((world_x, world_y), pose) {
                                state.dragging_modify = Some(ModifyDragTarget::StartBody);
                                target_found = true;
                                println!("Dragging Start Body");
                            }
                        }
                    }
                    if !target_found {
                        if let Some(ref pose) = state.end_pose {
                            if state.check_body_hit((world_x, world_y), pose) {
                                state.dragging_modify = Some(ModifyDragTarget::EndBody);
                                // target_found = true; // No need to set flag here
                                println!("Dragging End Body");
                            }
                        }
                    }
                }

                // Handle modification dragging
                if let Some(target) = state.dragging_modify {
                    if is_mouse_button_down(MouseButton::Left) {
                        match target {
                            ModifyDragTarget::StartBody => {
                                if let Some(pose) = &mut state.start_pose {
                                    pose.x = world_x;
                                    pose.y = world_y;
                                }
                            }
                            ModifyDragTarget::EndBody => {
                                if let Some(pose) = &mut state.end_pose {
                                    pose.x = world_x;
                                    pose.y = world_y;
                                }
                            }
                            ModifyDragTarget::StartAngle => {
                                if let Some(pose) = &mut state.start_pose {
                                    let dx = world_x - pose.x;
                                    let dy = world_y - pose.y;
                                    if dx.hypot(dy) > 1e-6 {
                                        pose.theta_degree = dy.atan2(dx).to_degrees();
                                    }
                                }
                            }
                            ModifyDragTarget::EndAngle => {
                                if let Some(pose) = &mut state.end_pose {
                                    let dx = world_x - pose.x;
                                    let dy = world_y - pose.y;
                                    if dx.hypot(dy) > 1e-6 {
                                        pose.theta_degree = dy.atan2(dx).to_degrees();
                                    }
                                }
                            }
                        }
                        // Recalculate paths continuously while modifying
                        state.calculate_and_prepare_paths();
                    } else if is_mouse_button_released(MouseButton::Left) {
                        println!("Modification finished.");
                        state.dragging_modify = None;
                        // Paths were already updated
                    }
                }
            }
        }

        // --- Drawing ---
        clear_background(DARKGRAY);

        // Draw paths first
        draw_paths(&state);

        // Draw car representations (body + headlight)
        if let Some(ref pose) = state.start_pose {
            draw_pose_elements(pose, BLUE);
        }
        if let Some(ref pose) = state.end_pose {
            draw_pose_elements(pose, RED);
        }

        // Draw UI elements (includes initial angle drag line)
        draw_ui(&state);

        next_frame().await
    }
}

