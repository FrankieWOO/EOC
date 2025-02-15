%% Import data from spreadsheet
% Script for importing data from the following spreadsheet:
%
%    Workbook: C:\workspace\EOC\plots\regebrake.xlsx
%    Worksheet: Sheet1
%
% To extend the code for use with different selected data or a different
% spreadsheet, generate a function instead of a script.

% Auto-generated by MATLAB on 2017/08/09 17:41:03

%% Import the data
[~, ~, raw] = xlsread('regebrake.xlsx','Sheet1','A2:D25');

%% Create output variable
data = reshape([raw{:}],size(raw));

%% Allocate imported array to column variable names
D1 = data(1:13,1);
D2 = data(14:end,1);
Td = data(:,2);
Prege = data(:,3);
damping = data(:,4);

%% Clear temporary variables
clearvars data raw;

%%
figure
subplot(1,2,1)
u = [D1; 1+D2];
yyaxis left
plot(u, damping)
yyaxis right
plot(u, Prege)

subplot(1,2,2)
plot(damping, Prege)