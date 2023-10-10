clc
clear all
close all

measurement = readmatrix("measure.csv");
hold on
plot(measurement(:,1), measurement(:,2));
plot(measurement(:,1), measurement(:,3));
hold off