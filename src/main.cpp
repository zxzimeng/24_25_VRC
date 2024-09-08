#include "main.h"

// Motor and IMU ports
#define LEFT_MOTORS {1, -2, 3}
#define RIGHT_MOTORS {-4, 5, -6}
#define IMU_PORT 16 // IMU port

// Define the localizer using the VOSS library
auto odom = voss::localizer::TrackingWheelLocalizerBuilder::new_builder()
				.with_left_motor(1)
				.with_right_motor(-4)
				.with_track_width(11)		// Adjust based on your robot's configuration
				.with_left_right_tpi(18.43) // Adjust TPI based on your calibration
				.with_imu(IMU_PORT)
				.build();

// Exit Conditions
auto ec = voss::controller::ExitConditions::new_conditions()
			  .add_settle(400, 0.5, 400)
			  .add_tolerance(1.0, 2.0, 200)
			  .add_timeout(22500)
			  .add_thru_smoothness(4)
			  .build();

// Define PID Controller
auto pid = voss::controller::PIDControllerBuilder::new_builder(odom)
			   .with_linear_constants(20, 0.02, 169)
			   .with_angular_constants(250, 0.05, 2435)
			   .with_min_error(5)
			   .with_min_vel_for_thru(100)
			   .build();

// Define DiffChassis
auto chassis = voss::chassis::DiffChassis(LEFT_MOTORS, RIGHT_MOTORS, pid, ec, 8, pros::E_MOTOR_BRAKE_COAST);

void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);

	// Start odometry localization
	// odom->begin_localization();
	pros::delay(3000); // Wait for 3 seconds
}

void disabled() {}

void competition_initialize() {}

void autonomous()
{
	// Add autonomous code here using VOSS library
}

double applyJoystickCurve(double input)
{
	// Normalize the input to the range [-1, 1]
	double normalizedInput = input / 100.0; // Scale input from [-100, 100] to [-1, 1]

	// Apply a quadratic curve for joystick input mapping
	double mappedSpeed = std::pow(normalizedInput, 2) * (normalizedInput >= 0 ? 0.4 : -0.4);

	// Map speed to the range [-600, 600]
	return mappedSpeed * 600;
}

void opcontrol()
{
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor motor(17, pros::v5::MotorGears::green); // Replace 17 with your motor port
	pros::adi::Pneumatics adi1('a', false);				// Replace 'a' with your first ADI port

	// Configure ADI devices for piston control
	adi1.retract(); // Set initial piston position (retracted)
	bool mogo_piston_state = false;
	bool intake_state = false;
	bool reverse_intake_state = false;

	std::uint32_t last_toggle_time = 0; // Store the last toggle time

	// Initialize motor groups
	pros::MotorGroup leftMotors({-14, 15, 16});
	pros::MotorGroup rightMotors({-11, 12, -13});
	leftMotors.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);

	while (true)
	{
		// Motor control
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			if (pros::millis() - last_toggle_time >= 200)
			{ // Check if 200 milliseconds have passed
				intake_state = !intake_state;
				last_toggle_time = pros::millis();
			}
		}

		// Motor control
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			if (pros::millis() - last_toggle_time >= 200)
			{ // Check if 200 milliseconds have passed
				reverse_intake_state = !reverse_intake_state;
				last_toggle_time = pros::millis();
			}
		}

		// Piston control
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			if (pros::millis() - last_toggle_time >= 200)
			{ // Check if 200 milliseconds have passed
				mogo_piston_state = !mogo_piston_state;
				last_toggle_time = pros::millis();
			}
		}

		if (intake_state)
		{
			motor.move_voltage(-12000);
		}
		else if (!reverse_intake_state)
		{
			motor.move_voltage(0);
		}

		if (reverse_intake_state)
		{
			motor.move_voltage(12000);
		}
		else if (!intake_state)
		{
			motor.move_voltage(0);
		}

		if (mogo_piston_state)
		{
			adi1.extend();
		}
		else
		{
			adi1.retract();
		}

		// Get joystick inputs
		double forwardBack = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); // Forward/backward input
		double rotation = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);	  // Rotation input

		// Apply joystick curve
		double leftSpeed = applyJoystickCurve(forwardBack + rotation);
		double rightSpeed = applyJoystickCurve(forwardBack - rotation);

		// Set motor velocities (±600 RPM)
		leftMotors.move_velocity(leftSpeed);
		rightMotors.move_velocity(rightSpeed);

		pros::delay(20);
	}
}