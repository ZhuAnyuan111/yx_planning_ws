def angle_diff2normal_area(angle_diff):
    return (angle_diff + 180) % 360 - 180



if __name__ == '__main__':
    test_angle = angle_diff2normal_area(-190)
    print(test_angle)

