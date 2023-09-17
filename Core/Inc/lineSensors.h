/*
 * lineSensors.h
 *
 *  Created on: Sep 16, 2023
 *      Author: gabriel
 */

#ifndef INC_LINESENSORS_H_
#define INC_LINESENSORS_H_

typedef enum {
	black,
	white
} color;

typedef struct {
	color mostLeftSensor;
	color leftSensor;
	color middleSensor;
	color rightSensor;
	color mostRightSensor;
} sensorsStateStruct;

sensorsStateStruct xLineSensorsGetState();

#endif /* INC_LINESENSORS_H_ */
