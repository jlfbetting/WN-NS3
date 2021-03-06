% MATLAB FILE TO PROCESS NS3 OUTPUT FILES
% FOR CHAPTER 1 AND 2 OF THE REPORT,
% THAT WERE GENERATED BY THE ASSIGNMENT 
% SCRIPT, AS WRITTEN BY J.L.F. BETTING FOR
% THE WIRELESS NETWORKING COURSE AT TU DELFT

% VARIABLES
filename = 'myFile.out'; % change to title of input file

maxDistance = 100; % change according to parameters of input file.
step = 0.5;
simDataText = 'Path loss in het Hybrid Buildings Propagation Loss Model, situation 2';

delimiter = ' ';
A = importdata(filename,delimiter,0);
A = A([1:2*(maxDistance/step)+1],[1:2*(maxDistance/step)+1]);
Distance = 0:step:(2*maxDistance);
Distance = Distance - maxDistance;
middleRow = A(maxDistance+1,:);
figure(1);
surf(A, 'EdgeColor','None');
view(2);
colormap jet;
hold on;
set(gca,'XTickLabel',{})
set(gca,'YTickLabel',{})
xlim([1,2*(maxDistance/step)+1]);
ylim([1,2*(maxDistance/step)+1]);
xlabel('Distance');
ylabel('Distance');

title(simDataText);

figure(2)
plot(Distance,middleRow);
hold on;
title(simDataText);
xlabel('Distance from access point [m]');
ylabel('Signal power loss in dBm');
xlim([-maxDistance,maxDistance]);

