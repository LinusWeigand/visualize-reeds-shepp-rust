use macroquad::prelude::*;
use reeds_shepp_lib::{Gear, Path, PathElement, Pose, Steering, get_all_paths, get_optimal_path};
use std::f64::consts::PI;

const WINDOW_WIDTH: i32 = 1024;
const WINDOW_HEIGHT: i32 = 768;
const CAR_WIDTH: f32 = 30.0;
const CAR_LENGTH: f32 = 50.0;
const PATH_RESOLUTION: f64 = 20.0;
const TURNING_RADIUS: f64 = 1.0;
const DRAW_SCALE: f32 = 50.0;
const BODY_HIT_RADIUS_WORLD: f64 = (CAR_LENGTH / DRAW_SCALE / 2.0) as f64;
const HEADLIGHT_SIZE_SCREEN: f32 = 8.0;
const HEADLIGHT_HIT_RADIUS_WORLD: f64 = (HEADLIGHT_SIZE_SCREEN / DRAW_SCALE * 1.5) as f64;
const BEAM_LENGTH: f32 = 60.0; // Length of headlight beam
const BEAM_WIDTH: f32 = 40.0; // Width of headlight beam at the end
const TURNING_CIRCLE_OPACITY: f32 = 0.2; // Opacity for turning radius circles

// Colors matching the image
const START_CAR_COLOR: Color = Color::new(0.7, 0.9, 0.7, 1.0); // Light green
const END_CAR_COLOR: Color = Color::new(0.4, 0.5, 0.9, 1.0); // Blue
const OPTIMAL_PATH_COLOR: Color = Color::new(1.0, 0.6, 0.1, 1.0); // Orange
const HEADLIGHT_COLOR: Color = Color::new(1.0, 1.0, 0.7, 1.0); // Light yellow
const BEAM_COLOR: Color = Color::new(1.0, 1.0, 0.5, 0.5); // Yellow with transparency
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
            if (drag.current_pos - drag.start_pos).length_squared() > 10.0 * 10.0 {
                let angle_rad_screen = (drag.current_pos.y - drag.start_pos.y)
                    .atan2(drag.current_pos.x - drag.start_pos.x);
                return Some((-angle_rad_screen).to_degrees().into());
            }
        }
        None
    }

    fn check_body_hit(&self, world_click_pos: (f64, f64), pose: &Pose) -> bool {
        let dx = world_click_pos.0 - pose.x;
        let dy = world_click_pos.1 - pose.y;
        let dist_sq = dx * dx + dy * dy;
        dist_sq < BODY_HIT_RADIUS_WORLD * BODY_HIT_RADIUS_WORLD
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
        dist_sq < HEADLIGHT_HIT_RADIUS_WORLD * HEADLIGHT_HIT_RADIUS_WORLD
    }

    fn calculate_and_prepare_paths(&mut self) {
        if let (Some(start), Some(end)) = (self.start_pose, self.end_pose) {
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
            self.all_paths_points.clear();
            self.optimal_path_points = None;
        }
    }
}

fn path_length(path: &Path) -> f64 {
    path.iter().map(|e| e.param.abs()).sum()
}

fn generate_path_points(
    start_pose: &Pose,
    end_pose: &Pose,
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
                Steering::Left => {
                    let angle_step = step_size * gear_mult / TURNING_RADIUS;
                    (
                        TURNING_RADIUS * (current_theta_rad + angle_step).sin()
                            - TURNING_RADIUS * current_theta_rad.sin(),
                        TURNING_RADIUS * current_theta_rad.cos()
                            - TURNING_RADIUS * (current_theta_rad + angle_step).cos(),
                        angle_step,
                    )
                }
                Steering::Right => {
                    let angle_step = step_size * gear_mult / TURNING_RADIUS;
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
    points
}

// Draw car with headlight beam to match the image
fn draw_pose_elements(pose: &Pose, body_color: Color) {
    let center_screen = State::world_to_screen_static(pose.x, pose.y);
    let rotation_rad_world = pose.theta_degree.to_radians();
    let rotation_rad_screen = -rotation_rad_world as f32;

    // Draw headlight beam first (behind car)
    let (headlight_x_world, headlight_y_world) = State::get_headlight_world_pos(pose);
    let headlight_screen = State::world_to_screen_static(headlight_x_world, headlight_y_world);

    // Calculate beam vertices (trapezoid shape)
    let beam_direction = Vec2::new(
        rotation_rad_world.cos() as f32,
        -rotation_rad_world.sin() as f32,
    )
    .normalize();

    let beam_normal = Vec2::new(-beam_direction.y, beam_direction.x);

    // Start position (at headlight)
    let beam_start1 = headlight_screen + beam_normal * (HEADLIGHT_SIZE_SCREEN * 0.5);
    let beam_start2 = headlight_screen - beam_normal * (HEADLIGHT_SIZE_SCREEN * 0.5);

    // End position (wider)
    let beam_end1 =
        headlight_screen + beam_direction * BEAM_LENGTH + beam_normal * (BEAM_WIDTH * 0.5);
    let beam_end2 =
        headlight_screen + beam_direction * BEAM_LENGTH - beam_normal * (BEAM_WIDTH * 0.5);

    // Draw beam as polygon
    draw_triangle(beam_start1, beam_end1, beam_end2, BEAM_COLOR);
    draw_triangle(beam_start1, beam_end2, beam_start2, BEAM_COLOR);

    // Draw body
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

    // Draw turning radius circle (behind the car)
    let turning_center_world_x = pose.x - TURNING_RADIUS * rotation_rad_world.sin() as f64;
    let turning_center_world_y = pose.y + TURNING_RADIUS * rotation_rad_world.cos() as f64;
    let turning_center_screen =
        State::world_to_screen_static(turning_center_world_x, turning_center_world_y);
    let turning_radius_screen = TURNING_RADIUS as f32 * DRAW_SCALE;

    draw_circle_lines(
        turning_center_screen.x,
        turning_center_screen.y,
        turning_radius_screen,
        1.0,
        TURNING_CIRCLE_COLOR,
    );

    // Draw a small dot at the turning center
    draw_circle(
        turning_center_screen.x,
        turning_center_screen.y,
        2.0,
        TURNING_CIRCLE_COLOR,
    );

    // Draw headlight (red circle with yellow inner circle as in the image)
    draw_circle(
        headlight_screen.x,
        headlight_screen.y,
        HEADLIGHT_SIZE_SCREEN,
        RED,
    );

    // Inner headlight circle
    draw_circle(
        headlight_screen.x,
        headlight_screen.y,
        HEADLIGHT_SIZE_SCREEN * 0.6,
        HEADLIGHT_COLOR,
    );
}

// Draw the calculated paths
fn draw_paths(state: &State) {
    // Draw all paths (thin, semi-transparent light gray as in the image)
    for path_points in &state.all_paths_points {
        if path_points.len() > 1 {
            for i in 0..(path_points.len() - 1) {
                draw_line(
                    path_points[i].x,
                    path_points[i].y,
                    path_points[i + 1].x,
                    path_points[i + 1].y,
                    1.0,
                    Color::new(0.8, 0.8, 0.8, 0.4),
                );
            }
        }
    }

    // Draw optimal path (thicker orange as in the image)
    if let Some(optimal_points) = &state.optimal_path_points {
        if optimal_points.len() > 1 {
            for i in 0..(optimal_points.len() - 1) {
                draw_line(
                    optimal_points[i].x,
                    optimal_points[i].y,
                    optimal_points[i + 1].x,
                    optimal_points[i + 1].y,
                    3.0,
                    OPTIMAL_PATH_COLOR,
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
                        theta_degree: 0.0,
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
                        theta_degree: 0.0,
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
                        println!("End angle set to: {:.1}°", end.theta_degree);
                    }
                    state.drag_state_initial = None;
                    state.calculate_and_prepare_paths();
                    state.app_state = AppState::DisplayingPaths;
                    println!("Poses defined. Displaying paths. Drag to modify.");
                }
            }
            AppState::DisplayingPaths => {
                if is_mouse_button_pressed(MouseButton::Left) {
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
                                println!("Dragging End Body");
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
                        state.calculate_and_prepare_paths();
                    } else if is_mouse_button_released(MouseButton::Left) {
                        println!("Modification finished.");
                        state.dragging_modify = None;
                    }
                }
            }
        }

        // --- Drawing ---
        clear_background(Color::new(0.2, 0.2, 0.2, 1.0)); // Darker background like in the image

        // Draw turning circles and paths first
        draw_paths(&state);

        // Draw car representations
        if let Some(ref pose) = state.start_pose {
            draw_pose_elements(pose, START_CAR_COLOR);
        }
        if let Some(ref pose) = state.end_pose {
            draw_pose_elements(pose, END_CAR_COLOR);
        }

        // Draw UI elements
        draw_ui(&state);

        next_frame().await
    }
}

