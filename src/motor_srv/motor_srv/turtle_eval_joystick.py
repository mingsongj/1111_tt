import argparse
import os
os.environ['PYOPENGL_PLATFORM'] = 'glx'
import pickle
import pygame  # For joystick input
import torch
from turtle_env import Go2Env
from rsl_rl.runners import OnPolicyRunner
import genesis as gs

def get_joystick_input():
    """
    Reads joystick input and maps it to desired values for control inputs.
    Returns a tuple (A_x, A_y, ang_vel) based on joystick axis positions.
    """
    pygame.event.pump()  # Process joystick events
    x_axis = joystick.get_axis(0)  # Axis 0: linear velocity x
    y_axis = joystick.get_axis(1)  # Axis 1: linear velocity y
    angular_axis = joystick.get_axis(3)  # Axis 3: angular velocity

    # Map joystick values to control input ranges
    A_x = 0.8 * y_axis  # Map x-axis to [-0.6, 0.6]
    A_y = 0.6 * x_axis  # Map y-axis to [-0.2, 0.2]
    ang_vel = 1 * angular_axis  # Map angular velocity to [-0.3, 0.3]
    return A_x, A_y, ang_vel

def get_button_presses():
    """
    Checks for button presses on the joystick and returns a list of pressed buttons.
    """
    pressed_buttons = []
    for i in range(joystick.get_numbuttons()):
        if joystick.get_button(i):
            pressed_buttons.append(i)
    return pressed_buttons

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="turtle1-walking")
    parser.add_argument("--ckpt", type=int, default=800)
    args = parser.parse_args()

    gs.init()

    # Initialize joystick
    pygame.init()
    pygame.joystick.init()
    global joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    log_dir = f"logs/{args.exp_name}"
    env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg = pickle.load(open(f"{log_dir}/cfgs.pkl", "rb"))
    reward_cfg["reward_scales"] = {}

    env = Go2Env(
        num_envs=1,
        env_cfg=env_cfg,
        obs_cfg=obs_cfg,
        reward_cfg=reward_cfg,
        command_cfg=command_cfg,
        show_viewer=True,
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device="cuda:0")
    resume_path = os.path.join(log_dir, f"model_{args.ckpt}.pt")
    runner.load(resume_path)
    policy = runner.get_inference_policy(device="cuda:0")

    obs, _ = env.reset()
    # with torch.no_grad():
    #     while True:
    #         # Get joystick input
    #         A_x, A_y, ang_vel = get_joystick_input()
    #         pressed_buttons = get_button_presses()

    #         # Print joystick input and button presses
    #         print(f"Joystick Input - X: {A_x:.2f}, Y: {A_y:.2f}, Angular: {ang_vel:.2f}")
    #         if pressed_buttons:
    #             print(f"Buttons Pressed: {pressed_buttons}")

    #         # Update command configuration
    #         env.command_cfg["lin_vel_x_range"] = [A_x, A_x]  # Map axis 0 to x velocity range
    #         env.command_cfg["lin_vel_y_range"] = [A_y, A_y]  # Map axis 1 to y velocity range
    #         env.command_cfg["ang_vel_range"] = [ang_vel, ang_vel]  # Map axis 3 to angular velocity range

    #         # Perform simulation step
    #         actions = policy(obs)
    #         obs, _, rews, dones, infos = env.step(actions)


    with torch.no_grad():
        while True:
            # Get joystick input
            A_x, A_y, ang_vel = get_joystick_input()

            # Update commands directly in the environment
            env.commands[:, 0] = A_x  # Linear velocity x
            env.commands[:, 1] = A_y  # Linear velocity y
            env.commands[:, 2] = ang_vel  # Angular velocity

            # Perform simulation step
            actions = policy(env.get_observations())  # Use updated observations with joystick commands
            obs, _, rews, dones, infos = env.step(actions)

            # Reset environment if needed
            if dones[0]:  # Assuming single environment
                obs, _ = env.reset()

if __name__ == "__main__":
    main()
