#ifndef MPU_H
#define MPU_H

struct s_mympu {
	float ypr[4];
	float gyro[3];
  float accel[3];
};


extern struct s_mympu mympu;

int mympu_open(unsigned int rate);
int mympu_update();

#endif
