#ifndef ADNS3080_H_
#define ADNS3080_H_

#define ADNS3080_ADDRESS

#define ADNS3080_REG_MOTION 		0x02
#define ADNS3080_REG_DELTA_X 		0x03
#define ADNS3080_REG_DELTA_Y 		0x04
#define ADNS3080_REG_SQUAL 			0x05
#define ADNS3080_REG_PIXEL_SUM 		0x06
#define ADNS3080_REG_MAXIMUM_PIXEL 	0x07

extern void adns3080_init();

#endif /* ADNS3080_H_ */
