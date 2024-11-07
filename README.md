# template-talon_standard

This template standard contains code for two main types of sensors, photo-electric, and gyroscopes.

This code template will explain how to use beam break and color sensors.

The first type of sensor, photo-electric:

Photo-electric sensors work by emitting a laser to a surface. Once an element emits the light, the other element tries to receive it. If the component gets the light, that means that nothing is obstructing the light, in other words, no object. If there is an object, the object will block the light, and the receiver will never get the light, meaning that there is an object.

Example Diagram:
![image](https://github.com/user-attachments/assets/e8e93da7-e164-4a0c-b597-9b4215b70131)

Important Point:
All photoelectric sensors can be tuned to be more sensitive or less sensitive, this can be done with a flathead which moves a little screw on the photoelectric sensor. You can change your sensitivity based on how you want the sensor to detect objects.

Overall Code Explanation:
You need two main constants for photoelectric sensors: the DIO port and if the sensor is flipped. These are two constants that you need to keep track of.

DIO port:
![image](https://github.com/user-attachments/assets/b5645af8-ea9a-4c41-9518-d5738aec36a7)

You can keep the default boolean value flipped to 'false' and switch it later if necessary. You should log if your sensor detects something and if the sensor is flipped periodically (public void periodic). You can test if the sensor is flipped by reading the value when you put your hand in front of it. If it's false, then you keep the 'flipped' boolean value to true. If it reads true, then you are fine. 

After you have figured out your flipped value from now on when you call your getter that finds out if there is an object in front of the sensor you just flip the value if needed. 

Fun Fact: DONT DONT DONT use i2c ports ever as it can cause robo-rio shutdowns completely.

// Next Section //

The second type of sensor, gyroscope:

This will not be a very in-depth explanation as more will be covered in 'level-2-drive'.

A gyroscope is a sensor that helps determine the robot's orientation/position in real-time. It is really helpful and is needed on all robots. 

Here is the specific gyroscope we use. (read more about it ): 

![image](https://github.com/user-attachments/assets/38a4fdcb-ba37-460e-9580-fb5dbc55db7e)

The main value we pull from gyroscopes is the 'yaw' which is displayed below:

![image](https://github.com/user-attachments/assets/69521094-8e2e-4336-8e42-8a77d70ddd7e)

Other values are pulled from the gyroscope but you should only need 'yaw' unless we do a balance game (Charged Up -> Look it up)

The ID of this gyroscope can be found in Phoenix Tuner X.

The default value for how the yaw comes out is in degrees, meaning that for logging values, you usually need to 'modulus' by 360 so that the rotations don't build up.

Gyroscopes also need to be in the dead center of the robot. This is done by taking a string from the middle of the robot's horizontal and vertical sides. The intersection of the strings is the exact center of the robot. There is a little icon on the pigeon that can help indicate this 'dead center'. If you are to use strings and you see the icon is covered by the pigeon then you are good to go.

Gyroscopes can also be inverted sometimes, the way to check is to rotate the robot counter clockwise. If the yaw increases then your gyro does not need to be inverted.

READ THE DOCUMENTATION I KEPT IN THE CODE
