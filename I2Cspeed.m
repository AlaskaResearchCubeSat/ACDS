time=[294.5e-6 385.0e-6 476.0e-6 566.5e-6 657.5e-6 748.5e-6 839e-6 930e-6 1.112e-3 1.202e-3 1.293e-3];
len= [0        1        2        3        4        5        6      7      8        10       11];

plot(len,time*1e3);
title('I2C transaction time');
ylabel('transaction time [ms]');
xlabel('payload [bytes]');