I2C_time=[294.5e-6 385.0e-6 476.0e-6 566.5e-6 657.5e-6 748.5e-6 839e-6 930e-6 1.112e-3 1.202e-3 1.293e-3 2.11e-3 3.02e-3 3.86e-3];
I2C_len= [0        1        2        3        4        5        6      7      8        10       11       20      30      40     ];

SPI_time=[1.275e-3 1.275e-3 1.28e-3 1.39e-3 1.505e-3 1.74e-3 2.515e-3 3.76e-3 5.01e-3 6.265e-3 6.385e-3];
SPI_len= [2        5        10      50      100      200     500      1000    1500    2000     2048];

figure(1);
plot(SPI_len,SPI_time*1e3,'b');
title('SPI Transaction time');
ylabel('transaction time [ms]');
xlabel('payload [bytes]');
axis('tight');

figure(2);
plot(I2C_len,I2C_time*1e3,'r',SPI_len,SPI_time*1e3,'b');
title('Transaction time');
ylabel('transaction time [ms]');
xlabel('payload [bytes]');
legend('I2C','SPI');
axis([0 max(I2C_len) 0 max(I2C_time*1e3)]);

SPI_rate=SPI_len./SPI_time*8;
I2C_rate=I2C_len./I2C_time*8;

figure(3);
plot(SPI_len,SPI_rate/1e3,'b');
title('SPI Effective bit Rate');
xlabel('payload [bytes]');
ylabel('Bit Rate [kbps]');
axis('tight');

figure(4);
plot(I2C_len,I2C_rate/1e3,'r',SPI_len,SPI_rate/1e3,'b');
title('Effective bit Rate');
xlabel('payload [bytes]');
ylabel('Bit Rate [kbps]');
legend('I2C','SPI');
axis([0 max(I2C_len) 0 max(I2C_rate/1e3)]);