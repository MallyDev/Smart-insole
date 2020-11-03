# Smart insole
### Embedded system project by Maria Laura Bisogno, Antonio Nappi, Vittorio Senatore

The aim of this project was to build a smart insole to check the weight distribution and the walk of the person who wears it. This system could be used:
- to collect the data about the patient and analyse the progress during physical therapies;
- as diagnostic tool to analyse daily the patient’s condition;
- as prevention tool to prevent diseases caused by the incorrect posture of feet.

When the user wears the insoles for the first time, the system needs to be initialized with the values of user’s weight distribution. This can be simply done wearing the insoles, standing as usual and pressing the blue button on the controller. In this way the system will be ready to collect the data about the user. 
The analysis is both static and dynamic.
Static analysis: the insoles check if the weight distributed on the calcaneus is equal to the one distributed on the top of the first, fourth and fifth metatarsal;
Dynamic analysis: the insoles check if the user makes all the correct steps to walk and check the rotation of the ankle.

![alt text](https://github.com/MallyDev/Smart-insole/blob/master/walk.jpg)

### Hardware
To realize the project, we had:
- NUCLEO-F401RE STM32 microcontroller;
- Keystudio HM-10 Bluetooth module;
- 4 FlexiForce A401;
- ADXL335 accelerometer;
- RGB led;
- Vibrating motor.

The Bluetooth module is used to send the data from the insoles to the mobile app.
The four FlexiForce sensors are used to check the weight distribution. Those have been collocated where the weight distribution is normally concentrated:
![alt text](https://github.com/MallyDev/Smart-insole/blob/master/foot.png)


The sensors are connected with the following configuration:

![alt text](https://github.com/MallyDev/Smart-insole/blob/master/force.png)

The accelerometer is used to check the rotation of the ankle. It is connected with the following configuration:

![alt text](https://github.com/MallyDev/Smart-insole/blob/master/acc.png)

The RGB led is used to give a visual feedback about the analysis:
- Green color: the user is walking correctly;
- Red color: the user is not walking correctly;
- Blue color: the user’s weight is distributed correctly;
- Yellow color: the user’s weight is not distributed correctly.

A vibrating motor is also used to give physical feedback to the user when he/she doesn’t walk correctly.

The total system appears like this:

![alt text](https://github.com/MallyDev/Smart-insole/blob/master/total.png)



### Software
The code is in C language and follows the interrupt driven model.
