"""
Author: Vamsi Kalagaturu

Description:
This script plots the data from the logs.
"""

import os
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd


class Plotter:

    def __init__(self) -> None:
        # set the output path to be the root of the project
        self.root_path = '/home/batsy/rnd/outputs/logs/data/'

        self.out_path = os.path.join(self.root_path, '../../plots')

        # create the output directory if it does not exist
        os.makedirs(self.out_path, exist_ok=True)

        # load the data
        self.load_data()

    def load_data(self):
        arm_down_vel_folders = [i for i in range(0, 14, 3)]
        arm_fwd_folders = [i for i in range(1, 14, 3)]
        arm_fwd_force_folders = [i for i in range(2, 14, 3)]

        self.arm_down_vel_data = []
        self.arm_fwd_data = []
        self.arm_fwd_force_data = []

        self.tasks = []

        for folder in arm_down_vel_folders:
            # get .csv file in the folder
            csv_file = [f
                        for f in os.listdir(
                            os.path.join(self.root_path, str(folder)))
                        if f.endswith('.csv')][0]
            # load the data
            data = pd.read_csv(os.path.join(
                self.root_path, str(folder), csv_file))
            # append to the list
            self.arm_down_vel_data.append(data)

        for folder in arm_fwd_folders:
            # get .csv file in the folder
            csv_file = [f
                        for f in os.listdir(
                            os.path.join(self.root_path, str(folder)))
                        if f.endswith('.csv')][0]
            # load the data
            data = pd.read_csv(os.path.join(
                self.root_path, str(folder), csv_file))
            # append to the list
            self.arm_fwd_data.append(data)

        for folder in arm_fwd_force_folders:
            # get .csv file in the folder
            csv_file = [f
                        for f in os.listdir(
                            os.path.join(self.root_path, str(folder)))
                        if f.endswith('.csv')][0]
            # load the data
            data = pd.read_csv(os.path.join(
                self.root_path, str(folder), csv_file))
            # append to the list
            self.arm_fwd_force_data.append(data)

        self.tasks.append(self.arm_down_vel_data)
        self.tasks.append(self.arm_fwd_data)
        self.tasks.append(self.arm_fwd_force_data)

    def plot(self):
        # plot the linear vel of z from 5 different runs in the same plot using seaborn
        # create a dataframe
        df = pd.DataFrame()
        for i in range(len(self.arm_down_vel_data)):
            df['run_' + str(i)] = self.arm_down_vel_data[i]['current_vel_lin_z']

        # plot the data
        sns.set_theme(style="darkgrid")

        # plot the average of the runs
        ax = sns.lineplot(data=df.mean(axis=1), label='Average')
        # plot the standard deviation of the runs
        ax.fill_between(
            df.index, df.mean(axis=1) - df.std(axis=1),
            df.mean(axis=1) + df.std(axis=1),
            alpha=0.2)
        # plot the individual runs
        # for i in range(len(self.arm_down_vel_data)):
        #     sns.lineplot(data=df['run_' + str(i)], ax=ax, label='run_' + str(i))

        # set the labels
        ax.set(
            xlabel='Time (s)', ylabel='Linear Velocity (m/s)',
            title='Linear Velocity of Z')
        # show the plot
        plt.show()

    def plot_velocity(
            self, ax, data, tdf, ylabel, title, target: bool = False,
            vertical_lines: bool = False):
        df = pd.DataFrame(data)
        avg = df.mean(axis=1)
        std = df.std(axis=1)

        sns.lineplot(data=avg, label='Average', ax=ax)
        ax.fill_between(df.index, avg - std, avg + std, alpha=0.2)
        if target:
            sns.lineplot(data=tdf.mean(axis=1), label='Target', ax=ax)
        ax.set(xlabel='Iterations', ylabel=ylabel, title=title)
        if vertical_lines:
            # plot vertical area between 2510 and 2560 using sns color palette
            ax.axvspan(
                2510, 2560, alpha=0.2, color=sns.color_palette()[2],
                label='Contact Detection')
            ax.legend()

    def plot_velocities(
            self, task: int = 0, target: bool = False, fig_title: str = ''):
        data = self.tasks[task]
        sns.set_theme(style="darkgrid")
        fig, axs = plt.subplots(3, 2, figsize=(10, 10))

        components = ['x', 'y', 'z']
        velocities = ['lin', 'ang']

        for comp in components:
            for vel in velocities:
                current_data_key = f'current_vel_{vel}_{comp}'
                target_data_key = f'target_vel_{vel}_{comp}'

                title_key = 'Linear' if vel == 'lin' else 'Angular'
                units = 'm/s' if vel == 'lin' else 'rad/s'

                title = f'{title_key} Velocity of {comp.upper()}'

                df = pd.DataFrame()
                tdf = pd.DataFrame()
                for i in range(1, len(data)):
                    df['run_' + str(i)] = data[i][current_data_key]
                    tdf['run_' + str(i)] = data[i][target_data_key]
                    # replace inf with 0
                    tdf = tdf.replace([np.inf, -np.inf], 0)

                vf = False
                if comp == 'z' and vel == 'lin' and task == 0:
                    vf = True

                self.plot_velocity(
                    axs[components.index(comp)]
                    [velocities.index(vel)],
                    df, tdf, f'{title_key} Velocity ({units})',
                    title, target, vf)

        plt.tight_layout()
        # set the title
        fig.suptitle(fig_title)
        plt.subplots_adjust(top=0.9)
        # save the plot
        plt.savefig(os.path.join(self.out_path, f'task_{task}_velocities.png'))
        plt.show()

    def plot_taus(self, task: int = 0, fig_title: str = ''):
        data = self.tasks[task]
        # plot constraint taus
        sns.set_theme(style="darkgrid")
        labels = [f'constraint_tau{i}' for i in range(1, 8)]

        # size of the plot
        plt.figure(figsize=(10, 10))

        # plot in single plot
        for i in range(len(labels)):
            df = pd.DataFrame()
            for j in range(1, len(data)):
                df['run_' + str(j)] = data[j][labels[i]]

            ax = sns.lineplot(
                data=df.mean(axis=1),
                label=f'constraint_torque_{i}')
            ax.fill_between(
                df.index, df.mean(axis=1) - df.std(axis=1),
                df.mean(axis=1) + df.std(axis=1),
                alpha=0.2)
            ax.set(xlabel='Iterations',
                   ylabel='Constraint Torque', title=labels[i])

        # set the title
        plt.title(fig_title)
        plt.tight_layout()
        # save the plot
        plt.savefig(os.path.join(self.out_path,
                    f'task_{task}_constraint_taus.png'))
        plt.show()

    def plot_contrl_signals(self, task: int = 0, fig_title: str = ''):
        data = self.tasks[task]
        # plot constraint taus
        sns.set_theme(style="darkgrid")
        labels = [f'control_signal{i}' for i in range(1, 7)]

        # size of the plot
        plt.figure(figsize=(10, 10))

        # plot in single plot
        for i in range(len(labels)):
            df = pd.DataFrame()
            for j in range(len(data)):
                df['run_' + str(j)] = data[j][labels[i]]

            ax = sns.lineplot(data=df.mean(axis=1), label=f'control_signal_{i}')
            ax.fill_between(
                df.index, df.mean(axis=1) - df.std(axis=1),
                df.mean(axis=1) + df.std(axis=1),
                alpha=0.2)
            ax.set(xlabel='Iterations',
                   ylabel='Control Signal', title=labels[i])

        # set the title
        plt.title(fig_title)
        plt.tight_layout()
        # save the plot
        plt.savefig(os.path.join(self.out_path,
                    f'task_{task}_control_signals.png'))
        plt.show()

    def plot_positions(self, task: int = 0, fig_title: str = ''):
        data = self.tasks[task]
        sns.set_theme(style="darkgrid")

        current_data_keys = ['current_pos_x', 'current_pos_y', 'current_pos_z']
        target_data_keys = ['target_pos_x', 'target_pos_y', 'target_pos_z']

        fig, axs = plt.subplots(3, 1, figsize=(10, 10))
        for i in range(1):
            cdf = data[i][current_data_keys]
            tdf = data[i][target_data_keys]

            # plot current and target positions
            for j in range(len(current_data_keys)):
                ax = sns.lineplot(
                    data=cdf[current_data_keys[j]],
                    label=current_data_keys[j],
                    ax=axs[j])
                ax = sns.lineplot(
                    data=tdf[target_data_keys[j]],
                    label=target_data_keys[j],
                    ax=axs[j])
                ax.set(
                    xlabel='Iterations', ylabel='Position (m)',
                    title=f'Position of {current_data_keys[j].split("_")[2].upper()}')

        # set the title
        fig.suptitle(fig_title)
        plt.tight_layout()
        # save the plot
        plt.savefig(os.path.join(self.out_path, f'task_{task}_positions.png'))
        plt.show()


if __name__ == '__main__':
    plotter = Plotter()
    # plotter.plot_velocities(task=2, target=False, fig_title='Partial Constraint Specifications (Linear XY) (Force 20N in -X) \n Average End-Effector Velocity (5 Runs)')
    # plotter.plot_taus(task=2, fig_title='Partial Constraint Specifications (Linear XY) (Force 20N in -X) \n Average Constraint Torques (5 Runs)')
    # plotter.plot_contrl_signals(task=1, fig_title='Partial Constraint Specifications (Linear XY) - Average Beta Energy (5 Runs)')
    plotter.plot_positions(
        task=1,
        fig_title='Partial Constraint Specifications (Linear XY) \n End-Effector Position')
