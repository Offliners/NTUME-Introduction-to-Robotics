import numpy as np
import matplotlib.pyplot as plt

(goal_x, goal_y) = (1, 1) 
class Two_Link_Manipulator:
    def __init__(self, l1, theta1, l2, theta2):
        self.J0_loc = np.array([0, 0])
        self.L1 = l1
        self.L2 = l2
        self.move(theta1, theta2)

    def move(self, next_theta1, next_theta2):
        self.J_angles = [next_theta1, next_theta2]
        self.FK()

    def FK(self):
        theta0 = self.J_angles[0]
        l1 = self.L1
        self.J1_loc = self.J0_loc + [l1 * np.cos(theta0), l1 * np.sin(theta0)]
        theta1 = self.J_angles[1]
        l2 = self.L2
        self.J2_loc = self.J1_loc + [l2 * np.cos(theta0 + theta1), l2 * np.sin(theta0 + theta1)]

    def plot(self):
        plt.cla()
        x = [self.J0_loc[0], self.J1_loc[0], self.J2_loc[0]]
        y = [self.J0_loc[1], self.J1_loc[1], self.J2_loc[1]]
        plt.plot(x, y, c="blue", zorder=1)
        plt.scatter(x, y, c="black", zorder=2)
        global goal_x,goal_y
        plt.scatter(goal_x, goal_y, c='green', marker='o')
        theta = np.linspace(0, 2 * np.pi, 100)
        r = self.L1 + self.L2
        a = r * np.cos(theta)
        b = r * np.sin(theta)
        plt.plot(a, b, c='red', label='Work Space')
        plt.xlim(-r, r)
        plt.ylim(-r, r)
        plt.legend(loc='lower right')
        plt.title('Two Link Manipulator Simulator')

    def IK(self, x, y):
        l1 = self.L1
        l2 = self.L2
        theta2 = np.arccos((x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2))
        theta1 = np.arctan2(y, x) - np.arctan2(l2 * np.sin(theta2), l2 * np.cos(theta2) + l1)
        return theta1, theta2

    def animation(self, x, y):
        theta1, theta2 = self.IK(x, y)
        duration_time_seconds = 1
        frames = 15
        angles_per_action = (np.array([theta1, theta2]) - np.array(self.J_angles)) / frames
        plt.ion()
        for _ in range(frames):
            self.J_angles = np.array(self.J_angles) + angles_per_action
            self.move(self.J_angles[0], self.J_angles[1])
            self.plot()
            dt = duration_time_seconds / frames
            plt.pause(dt)
    
    def mouse_click_posi(self, event):
        global goal_x, goal_y
        if event.xdata == None or event.ydata == None:
            return
        
        goal_x = event.xdata
        goal_y = event.ydata
    
        self.animation(goal_x, goal_y)

def main():
    fig = plt.figure(figsize=(6, 6))
    RR = Two_Link_Manipulator(1, 0, 1, 0)
    RR.animation(goal_x, goal_y)
    fig.canvas.mpl_connect("button_press_event", RR.mouse_click_posi)
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()
