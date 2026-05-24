#!/usr/bin/env python3
"""
PPO training script for the cleaning robot.

Run this AFTER starting the simulation:
  Terminal 1:  ros2 launch cleaning_robot_gazebo sim.launch.py
  Terminal 2:  python3 ppo_train.py

Training takes ~1-3 hours depending on hardware.
The best model is saved to /tmp/ppo_model/ppo_cleaning_robot.zip.
"""
import os
import time

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import (
    BaseCallback,
    CheckpointCallback,
    EvalCallback,
)
from stable_baselines3.common.monitor import Monitor

from cleaning_robot_algorithms.ppo_env import CleaningRobotEnv

MODEL_DIR  = '/tmp/ppo_model'
MODEL_PATH = os.path.join(MODEL_DIR, 'ppo_cleaning_robot')
LOG_PATH   = '/tmp/ppo_training.log'
TOTAL_TIMESTEPS = 300_000


class EpisodeLogCallback(BaseCallback):
    """Logs episode reward and length to the training log file."""

    def __init__(self):
        super().__init__()
        self._ep_rewards = []
        self._ep_start   = time.time()

    def _on_step(self) -> bool:
        infos = self.locals.get('infos', [])
        for info in infos:
            if 'episode' in info:
                ep = info['episode']
                line = (
                    f"timestep={self.num_timesteps} "
                    f"ep_reward={ep['r']:.2f} "
                    f"ep_len={ep['l']} "
                    f"elapsed={time.time()-self._ep_start:.0f}s\n"
                )
                with open(LOG_PATH, 'a') as f:
                    f.write(line)
                print(line, end='')
        return True


def main():
    os.makedirs(MODEL_DIR, exist_ok=True)

    print("Creating Gazebo environment — make sure sim.launch.py is running...")
    env      = Monitor(CleaningRobotEnv())
    eval_env = Monitor(CleaningRobotEnv())

    model = PPO(
        policy          = 'MlpPolicy',
        env             = env,
        n_steps         = 2048,
        batch_size      = 64,
        n_epochs        = 10,
        learning_rate   = 3e-4,
        gamma           = 0.99,
        ent_coef        = 0.01,   # entropy bonus encourages exploration
        verbose         = 1,
        tensorboard_log = '/tmp/ppo_tensorboard/',
    )

    callbacks = [
        EpisodeLogCallback(),
        # Save a checkpoint every 10 000 steps
        CheckpointCallback(
            save_freq       = 10_000,
            save_path       = MODEL_DIR,
            name_prefix     = 'ppo_ckpt',
        ),
        # Evaluate every 10 000 steps and keep the best model
        EvalCallback(
            eval_env,
            best_model_save_path = MODEL_DIR,
            log_path             = MODEL_DIR,
            eval_freq            = 10_000,
            n_eval_episodes      = 3,
            deterministic        = True,
        ),
    ]

    print(f"Starting training — {TOTAL_TIMESTEPS:,} timesteps")
    print(f"Progress log: {LOG_PATH}")
    print(f"Best model will be saved to: {MODEL_PATH}.zip\n")

    model.learn(total_timesteps=TOTAL_TIMESTEPS, callback=callbacks)

    model.save(MODEL_PATH)
    print(f"\nTraining complete. Model saved to {MODEL_PATH}.zip")

    env.close()
    eval_env.close()


if __name__ == '__main__':
    main()
