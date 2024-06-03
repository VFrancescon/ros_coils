import rospy
from ros_coils.msg import magField
import threading

def check_float_input(input_str):
    try:
        # Attempt to convert the input string to a float
        value = float(input_str)
        return value
    except (ValueError, IndexError):
        # Catch ValueError from float conversion and IndexError from split
        raise ValueError("Invalid input. Please enter a float with up to 1 decimal place.")


def cli_thread():
    # Create a publisher on the "/field" topic with the magField message type
    pub = rospy.Publisher('/field', magField, queue_size=10)
    
    while not rospy.is_shutdown():
        inputs = []
    
        for i in range(1, 4):
            while not rospy.is_shutdown():
                user_input = input(f"Enter number {i} (float up to 1 decimal place): ")
                try:
                    # Check for the "shutdown" command
                    if user_input == "shutdown":
                        rospy.signal_shutdown("User requested shutdown")
                        break  # Exit the inner loop
                    # Clean and check the user's input
                    user_input = check_float_input(user_input)
                    rospy.loginfo(f"Received: {user_input}")
                    inputs.append(user_input)
                    break  # Valid input was received; exit the inner loop
                except ValueError as e:
                    rospy.logwarn(str(e))
        
        # rospy.loginfo(f"Received numbers: {inputs}")
        
        # Create a magField message and populate it with the user inputs
        field_msg = magField()
        field_msg.bx = inputs[0]
        field_msg.by = inputs[1]
        field_msg.bz = inputs[2]
        field_msg.header.stamp = rospy.Time.now()
        # Publish the magField message on the "/field" topic
        pub.publish(field_msg)


def main():
    rospy.init_node('cli_listener', anonymous=True)
    print("This program listens for user commands to control a magnetic field.")
    print("To control the magnetic field, enter the field values for all three directions.")
    print("The field in any direction should not change by more than 10mT in one move.")
    print("If the magnitude change limit is exceeded, the node will need resetting.")
    print("If any supply is asked to supply more than 70\% of its maximum current, the node will need resetting.")
    print("All three directions must be inputted each time.")
    print('Enter "shutdown" to exit.')

    # Start the CLI thread
    thread = threading.Thread(target=cli_thread)
    thread.start()
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Main loop
        rate.sleep()

    # Ensure the thread is properly joined before exiting
    thread.join()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
