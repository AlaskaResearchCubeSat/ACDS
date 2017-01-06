#ifndef __LTC24XX_H
#define __LTC24XX_H

#define LTC24XX_EN      0x20
#define LTC24XX_PRE     0x80
#define LTC24xx_SGL     0x10
#define LTC24xx_SIGN    0x08
#define LTC24xx_CH_0    0x00
#define LTC24xx_CH_1    0x01
#define LTC24xx_EN2     0x80
#define LTC24xx_IM      0x40
#define LTC24xx_FA      0x20
#define LTC24xx_FB      0x10
#define LTC24xx_SPD     0x08
#define LTC24xx_GS2     0x04
#define LTC24xx_GS1     0x02
#define LTC24xx_GS0     0x01

#define LTC24xx_GAIN1       (0)
#define LTC24xx_GAIN4       (LTC24xx_GS0)
#define LTC24xx_GAIN8       (LTC24xx_GS1)
#define LTC24xx_GAIN16      (LTC24xx_GS1|LTC24xx_GS0)
#define LTC24xx_GAIN32      (LTC24xx_GS2)
#define LTC24xx_GAIN64      (LTC24xx_GS2|LTC24xx_GS0)
#define LTC24xx_GAIN128     (LTC24xx_GS2|LTC24xx_GS1)
#define LTC24xx_GAIN264     (LTC24xx_GS2|LTC24xx_GS1|LTC24xx_GS0)

//address for all LTC24XX ADC's
#define LTC24XX_GLOBAL_ADDR 0x77

  //CA1    CA0    ADDRESS
  //LOW    LOW    0X14
  //LOW    HIGH   0X16
  //LOW    FLOAT  0x15
  //HIGH   LOW    0x26
  //HIGH   HIGH   0x34
  //HIGH   FLOAT  0x27
  //FLOAT  LOW    0x17
  //FLOAT  HIGH   0x25
  //FLOAT  FLOAT  0x24

#endif
  