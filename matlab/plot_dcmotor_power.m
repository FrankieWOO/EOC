x = -2:0.01:2;
y1 = x.^2;
y2 = x;
y3 = y1 + y2;
ya = -2:0.01:2;
figure
hold on
plot(x,y1,x,y2,x,y3)
plot(x,zeros(size(x)))
plot(zeros(size(ya)),ya)
hold off