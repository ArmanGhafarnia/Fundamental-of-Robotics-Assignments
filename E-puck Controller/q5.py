from math import sin, cos, atan2, dist, pi as PI
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')

K_P, K_A, K_B, T = 2, 6.5, -0.5, 0.05
X_D, Y_D = 0, 0
R = 50
NUMBER_OF_SIMULATIONS = 10
NUMBER_OF_POINTS = 100


def controller(p, a, b, k_p=K_P, k_a=K_A, k_b=K_B):
    v = k_p * p
    w = k_a * a + k_b * b
    return v, w


def move(v, w, x, y, teta, t=T):
    x_dot = cos(-teta) * v
    y_dot = -sin(-teta) * v

    new_x = x + t * x_dot
    new_y = y + t * y_dot
    new_teta = teta + t * w

    return new_x, new_y, new_teta


def go_to_polar(x, y, teta):
    p = dist((X_D, Y_D), (x, y))
    a = -teta + atan2(Y_D - y, X_D - x)
    b = -teta - a
    return p, a, b


if __name__ == "__main__":

    for j in range(NUMBER_OF_SIMULATIONS):

        angle = (j * (2 * PI) / NUMBER_OF_SIMULATIONS) + 1
        starting_x, starting_y = R * cos(angle), R * sin(angle)

        # Get the starting Point Coordinates:
        current_x, current_y, current_teta = starting_x, starting_y, PI
        # add the starting point to the list of all points traveled by robot:
        xs, ys = [current_x], [current_y]
        # show the starting point in plot:
        plt.subplot(121)
        plt.scatter(current_x, current_y, marker='<', s=100, color='black')
        plt.subplot(122)
        plt.scatter(current_x, current_y, marker='<', s=100, color='black')

        # SIMULATE:
        for i in range(NUMBER_OF_POINTS):
            # get the polar Coordinates from Cartesian:
            p, a, b = go_to_polar(current_x, current_y, current_teta)

            # get the Motion Commands from the Controller:
            v, w = controller(p, a, b)

            # Move the Robot:
            current_x, current_y, current_teta = move(
                v, w, current_x, current_y, current_teta)

            # add the new Coordinates to the list:
            xs.append(current_x)
            ys.append(current_y)

        # Plot the Robot Trajectory:
        plt.subplot(121)
        plt.plot(xs, ys, color='black')
        plt.subplot(122)
        plt.plot((starting_x, current_x), (starting_y, current_y), color='black')

    plt.subplot(121)
    plt.xlim((-60, 60))
    plt.ylim((-60, 60))
    plt.xlabel(xlabel='X[cm]')
    plt.ylabel(ylabel='Y[cm]')
    plt.title(label='Robot\'s Simulated Trajectory', loc='center')

    plt.subplot(122)
    plt.xlim((-60, 60))
    plt.ylim((-60, 60))
    plt.xlabel(xlabel='X[cm]')
    plt.ylabel(ylabel='Y[cm]')
    plt.title(label='Best Route', loc='center')

    plt.show()
