use macroquad::prelude::*;
// Corrected import: removed unused PathElement
use reeds_shepp_lib::{Gear, Path, Pose, Steering, get_all_paths, get_optimal_path};
use std::f64::consts::PI;

const WINDOW_WIDTH: i32 = 1024;
const WINDOW_HEIGHT: i32 = 768;
const CAR_WIDTH: f32 = 30.0;
const CAR_LENGTH: f32 = 50.0;
const PATH_RESOLUTION: f64 = 20.0; // Number of points per unit distance/radian
const TURNING_RADIUS: f64 = 1.0; // Corresponds to the implicit radius in the calculations
const DRAW_SCALE: f32 = 50.0; // Scale world units for drawing

// Application states
#[derive(PartialEq, Debug, Clone, Copy)]
enum AppState {
    PlacingStart,
    DefiningStartAngle,
    PlacingEnd,
    DefiningEndAngle,
    DisplayingPaths,
}

// Store drag state for angle definition
struct DragState {
    start_pos: Vec2,
    current_pos: Vec2,
}

// Holds the application's current state
struct State {
    app_state: AppState,
    start_pose: Option<Pose>,
    end_pose: Option<Pose>,
    drag_state: Option<DragState>,
    all_paths_points: Vec<Vec<Vec2>>, // Points for drawing all paths
    optimal_path_points: Option<Vec<Vec2>>, // Points for drawing the optimal path
}

impl State {
    fn new() -> Self {
        State {
            app_state: AppState::PlacingStart,
            start_pose: None,
            end_pose: None,
            drag_state: None,
            all_paths_points: Vec::new(),
            optimal_path_points: None,
        }
    }

    fn reset(&mut self) {
        *self = State::new();
        println!("State reset.");
    }

    // Converts screen coordinates to world coordinates
    // Corrected: Explicitly convert f32 results to f64
    fn screen_to_world(&self, screen_pos: Vec2) -> (f64, f64) {
        (
            ((screen_pos.x - WINDOW_WIDTH as f32 / 2.0) / DRAW_SCALE) as f64,
            ((WINDOW_HEIGHT as f32 / 2.0 - screen_pos.y) / DRAW_SCALE) as f64,
        )
    }

    // Converts world coordinates to screen coordinates
    fn world_to_screen(&self, world_x: f64, world_y: f64) -> Vec2 {
        vec2(
            world_x as f32 * DRAW_SCALE + WINDOW_WIDTH as f32 / 2.0,
            WINDOW_HEIGHT as f32 / 2.0 - world_y as f32 * DRAW_SCALE,
        )
    }

    // Calculates the angle from a drag operation
    // Corrected: Explicitly convert f32 result to f64
    fn calculate_drag_angle(&self) -> Option<f64> {
        if let Some(drag) = &self.drag_state {
            if (drag.current_pos - drag.start_pos).length_squared() > 10.0 {
                let angle_rad = (drag.current_pos.y - drag.start_pos.y)
                    .atan2(drag.current_pos.x - drag.start_pos.x);
                // Convert the f32 result of to_degrees() to f64
                return Some((-angle_rad.to_degrees()) as f64);
            }
        }
        None
    }

    // Triggers path calculation and prepares drawing data
    fn calculate_and_prepare_paths(&mut self) {
        if let (Some(start), Some(end)) = (&self.start_pose, &self.end_pose) {
            println!("Calculating paths...");
            println!(
                "Start: ({:.2}, {:.2}, {:.1}°)",
                start.x, start.y, start.theta_degree
            );
            println!(
                "End:   ({:.2}, {:.2}, {:.1}°)",
                end.x, end.y, end.theta_degree
            );

            // NOTE: Creating new Pose instances here just in case the original
            // start/end pose might be mutated elsewhere, although it shouldn't be.
            // Cloning would be better if Pose implements Clone.
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

            println!("Found {} possible paths.", all_paths.len());

            self.all_paths_points.clear();
            for path in &all_paths {
                let points = generate_path_points(start, path, PATH_RESOLUTION);
                if !points.is_empty() {
                    self.all_paths_points.push(points);
                }
            }

            self.optimal_path_points = None;
            if let Some(optimal_path) = optimal_path_opt {
                let optimal_points = generate_path_points(start, &optimal_path, PATH_RESOLUTION);
                if !optimal_points.is_empty() {
                    println!(
                        "Optimal path length: {:.3}",
                        path_length(&optimal_path) // Assumes path_length helper exists
                    );
                    self.optimal_path_points = Some(optimal_points);
                } else {
                    println!("Warning: Optimal path found but generated no points.");
                }
            } else {
                println!("No optimal path found.");
            }

            self.app_state = AppState::DisplayingPaths;
        } else {
            eprintln!("Error: Cannot calculate paths without start and end poses.");
        }
    }
}

// Helper to calculate path length (ensure this aligns with your lib if needed)
fn path_length(path: &Path) -> f64 {
    path.iter().map(|e| e.param.abs()).sum()
}
// --- Path Interpolation ---
// Generates a list of screen points for drawing a path
fn generate_path_points(start_pose: &Pose, path: &Path, resolution: f64) -> Vec<Vec2> {
    let mut points = Vec::new();
    let mut current_x = start_pose.x;
    let mut current_y = start_pose.y;
    let mut current_theta_rad = start_pose.theta_degree.to_radians();

    let state = State::new(); // Temporary state to use coordinate conversion easily
    points.push(state.world_to_screen(current_x, current_y));

    for element in path {
        let param = element.param; // param is guaranteed >= 0 by PathElement::create
        let num_steps = ((param.abs() * resolution).ceil().max(1.0)) as usize;
        // step_size will be >= 0
        let step_size = param / num_steps as f64; // Can be angle or distance

        let gear_mult = match element.gear {
            Gear::Forward => 1.0,
            Gear::Backwards => -1.0,
        };

        for _ in 0..num_steps {
            let (dx, dy, dtheta) = match element.steering {
                Steering::Straight => {
                    // step_size is distance
                    (
                        step_size * gear_mult * current_theta_rad.cos(),
                        step_size * gear_mult * current_theta_rad.sin(),
                        0.0,
                    )
                }
                Steering::Left => {
                    // step_size is angle in radians
                    let angle_step = step_size * gear_mult; // Turn direction depends on gear
                    let radius = TURNING_RADIUS;
                    (
                        // Correct formula derived from geometry / trig identities
                        radius
                            * (current_theta_rad.cos() * angle_step.sin()
                                - current_theta_rad.sin() * (1.0 - angle_step.cos())),
                        radius
                            * (current_theta_rad.sin() * angle_step.sin()
                                + current_theta_rad.cos() * (1.0 - angle_step.cos())),
                        angle_step,
                    )
                }
                Steering::Right => {
                    // step_size is angle in radians
                    let angle_step = step_size * gear_mult; // Turn direction depends on gear
                    let radius = TURNING_RADIUS;
                    (
                        // Corrected formula derived from geometry / trig identities
                        radius
                            * (current_theta_rad.cos() * angle_step.sin()
                                + current_theta_rad.sin() * (1.0 - angle_step.cos())),
                        radius
                            * (current_theta_rad.sin() * angle_step.sin()
                                - current_theta_rad.cos() * (1.0 - angle_step.cos())),
                        -angle_step, // dtheta is negative for right turn
                    )
                }
            };

            current_x += dx;
            current_y += dy;
            current_theta_rad += dtheta;
            // Normalize angle (good practice)
            current_theta_rad = current_theta_rad.rem_euclid(2.0 * PI);

            points.push(state.world_to_screen(current_x, current_y));
        }
    }

    // Debug: Print final calculated pose vs target pose
    if let Some(end_pose) = state.end_pose.as_ref() {
        // Need state access or pass end_pose
        let final_theta_deg = current_theta_rad.to_degrees().rem_euclid(360.0);
        println!(
            "Interpolation End: x={:.3}, y={:.3}, th={:.2}",
            current_x, current_y, final_theta_deg
        );
        println!(
            "Target End Pose:   x={:.3}, y={:.3}, th={:.2}",
            end_pose.x,
            end_pose.y,
            end_pose.theta_degree.rem_euclid(360.0)
        );
        let dist_err = ((current_x - end_pose.x).powi(2) + (current_y - end_pose.y).powi(2)).sqrt();
        let angle_diff =
            (final_theta_deg - end_pose.theta_degree + 180.0).rem_euclid(360.0) - 180.0; // Angle diff in [-180, 180]
        println!(
            "End Pose Error: dist={:.3}, angle={:.2}",
            dist_err, angle_diff
        );
    }

    points
}

// --- Drawing Functions ---

fn draw_pose_rectangle(state: &State, pose: &Pose, color: Color) {
    let center_screen = state.world_to_screen(pose.x, pose.y);
    // Use f64 for angle calculation, convert to f32 for drawing
    let rotation_rad = -pose.theta_degree.to_radians();

    draw_rectangle_ex(
        center_screen.x,
        center_screen.y,
        CAR_LENGTH,
        CAR_WIDTH,
        DrawRectangleParams {
            offset: vec2(0.5, 0.5),
            rotation: rotation_rad as f32, // Convert rotation to f32 for drawing
            color,
            ..Default::default()
        },
    );

    // Draw a line indicating forward direction
    let forward_offset_x = (CAR_LENGTH / 2.0) * (rotation_rad as f32).cos(); // Use f32 angle
    let forward_offset_y = (CAR_LENGTH / 2.0) * (rotation_rad as f32).sin(); // Use f32 angle
    draw_line(
        center_screen.x,
        center_screen.y,
        center_screen.x + forward_offset_x,
        center_screen.y + forward_offset_y,
        2.0,
        WHITE,
    );
}

fn draw_paths(state: &State) {
    // Draw all paths (thin, semi-transparent)
    for path_points in &state.all_paths_points {
        if path_points.len() > 1 {
            for i in 0..(path_points.len() - 1) {
                draw_line(
                    path_points[i].x,
                    path_points[i].y,
                    path_points[i + 1].x,
                    path_points[i + 1].y,
                    1.0,
                    Color::new(0.8, 0.8, 1.0, 0.5),
                );
            }
        }
    }

    // Draw optimal path (thicker, opaque)
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

fn draw_ui(state: &State) {
    let text = match state.app_state {
        AppState::PlacingStart => "Click to place START position",
        AppState::DefiningStartAngle => "Drag and release to set START angle",
        AppState::PlacingEnd => "Click to place END position",
        AppState::DefiningEndAngle => "Drag and release to set END angle",
        AppState::DisplayingPaths => "Paths displayed. Press 'R' to Reset.",
    };
    draw_text(text, 20.0, 30.0, 24.0, WHITE);

    let mouse_pos_screen = mouse_position();
    let (mouse_x_world, mouse_y_world) =
        state.screen_to_world(vec2(mouse_pos_screen.0, mouse_pos_screen.1));
    let coord_text = format!("World Coords: ({:.2}, {:.2})", mouse_x_world, mouse_y_world);
    draw_text(&coord_text, 20.0, 60.0, 20.0, LIGHTGRAY);

    if let Some(drag) = &state.drag_state {
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
        let mouse_pos = vec2(mouse_position().0, mouse_position().1);
        let (world_x, world_y) = state.screen_to_world(mouse_pos);

        if is_key_pressed(KeyCode::R) {
            state.reset();
        }

        match state.app_state {
            AppState::PlacingStart => {
                if is_mouse_button_pressed(MouseButton::Left) {
                    state.start_pose = Some(Pose {
                        x: world_x,
                        y: world_y,
                        theta_degree: 0.0,
                    });
                    state.drag_state = Some(DragState {
                        start_pos: mouse_pos,
                        current_pos: mouse_pos,
                    });
                    state.app_state = AppState::DefiningStartAngle;
                    println!(
                        "Start position set at screen: ({:.1}, {:.1}), world: ({:.2}, {:.2})",
                        mouse_pos.x, mouse_pos.y, world_x, world_y
                    );
                }
            }
            AppState::DefiningStartAngle => {
                if is_mouse_button_down(MouseButton::Left) {
                    if let Some(drag) = &mut state.drag_state {
                        drag.current_pos = mouse_pos;
                        if let Some(angle_deg) = state.calculate_drag_angle() {
                            if let Some(start) = &mut state.start_pose {
                                start.theta_degree = angle_deg;
                            }
                        }
                    }
                } else if is_mouse_button_released(MouseButton::Left) {
                    if let Some(start) = &state.start_pose {
                        println!("Start angle set to: {:.1}°", start.theta_degree);
                    }
                    state.drag_state = None;
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
                    state.drag_state = Some(DragState {
                        start_pos: mouse_pos,
                        current_pos: mouse_pos,
                    });
                    state.app_state = AppState::DefiningEndAngle;
                    println!(
                        "End position set at screen: ({:.1}, {:.1}), world: ({:.2}, {:.2})",
                        mouse_pos.x, mouse_pos.y, world_x, world_y
                    );
                }
            }
            AppState::DefiningEndAngle => {
                if is_mouse_button_down(MouseButton::Left) {
                    if let Some(drag) = &mut state.drag_state {
                        drag.current_pos = mouse_pos;
                        if let Some(angle_deg) = state.calculate_drag_angle() {
                            if let Some(end) = &mut state.end_pose {
                                end.theta_degree = angle_deg;
                            }
                        }
                    }
                } else if is_mouse_button_released(MouseButton::Left) {
                    if let Some(end) = &state.end_pose {
                        println!("End angle set to: {:.1}°", end.theta_degree);
                    }
                    state.drag_state = None;
                    state.calculate_and_prepare_paths();
                }
            }
            AppState::DisplayingPaths => {
                // Input handled by 'R' key check above
            }
        }

        // --- Drawing ---
        clear_background(DARKGRAY);

        let origin_screen = state.world_to_screen(0.0, 0.0);
        draw_circle(origin_screen.x, origin_screen.y, 5.0, WHITE);
        draw_line(
            origin_screen.x,
            origin_screen.y,
            origin_screen.x + DRAW_SCALE,
            origin_screen.y,
            1.0,
            RED,
        ); // X+
        draw_line(
            origin_screen.x,
            origin_screen.y,
            origin_screen.x,
            origin_screen.y - DRAW_SCALE,
            1.0,
            LIME,
        ); // Y+

        if let Some(ref pose) = state.start_pose {
            draw_pose_rectangle(&state, pose, BLUE);
        }
        if let Some(ref pose) = state.end_pose {
            draw_pose_rectangle(&state, pose, RED);
        }
        if state.app_state == AppState::DisplayingPaths {
            draw_paths(&state);
        }
        draw_ui(&state);

        next_frame().await
    }
}
