clear all;

measurements = [21.8637929589789 20.2203506373265 20.9086354548582 19.4088297790472 19.7690201455967 18.2522305081561 20.0698322085498 20.9269808973863 18.9418239566577 19.7775154459492 19.2759185698247 19.1915917077952 20.9023934810928 20.0672670774844 20.7395932426423 20.3967753097481 20.9357915234428 17.7427960326353 18.9778771242300 19.8281151942768 20.3479121327860 21.1484325229735 19.3603762787404 19.3950525746836 19.7148449846203 20.6952204831622 20.1064347758691 21.1257016538631 19.2280680974510 20.4111336427935 20.3871884394671 18.3559922327388 20.2963888134685 21.7396936608370 18.8022147884121 19.3629187380481 18.9832420937512 21.1290679574493 19.4434859247075 20.7513579891163 18.3552401786798 18.1774477510084 20.9973557875997 20.5933062663922 20.2800742270737 20.9670675872049 20.0675910317724 20.1399051714884 19.1251076833658 20.0095055752150]';
%measurements = randn(1,50);
%scaleVector = (([1:length(measurements)]+3) .^ -1)*8;
%measurements = measurements + 20;
estimates = (1:length(measurements));
estimates(1) = 23;                              % initial estimate
errors = (1:length(measurements));
errors(1) = 3;
measErr = 2;
kalmanGain = (1:size(measurements,1));

for n = 2:length(measurements)
    kalmanGain(n) = errors(n-1) / (errors(n-1) + measErr);   % Calculate kalman gain
    
    estimates(n) = estimates(n-1) + kalmanGain(n)*(measurements(n-1) - estimates(n-1));
    %errors(n) = (measErr * errors(n-1)) / (measErr + errors(n-1));
    errors(n) = (1-kalmanGain(n)) * errors(n-1);
end

plot(measurements, 'LineWidth',1.5);
hold on;
plot(estimates, 'LineWidth', 1.5);
hold off;
%plot(errors);

legend('Measurements','Estimates'); 
xlabel('t'); ylabel('Values');
prettyPlotEx(7,5,'measurement.png');
