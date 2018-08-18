function myReg = myLinReg(x,y)
% Calculates the slope and intercept of the linear regression line using
% the least squares method outlined in ME 236 Course Pack

% Determining number of data points
n = length(x);

% Initializing variables
xy = 0;
xSq = 0;

% Determines sum of x*y and x^2 values
for i = 1:n
    xy = xy + x(i)*y(i);
    xSq = xSq + (x(i)^2);
end

% Calculates slope and intercept of linear regression
intercept = (sum(x)*xy - xSq*sum(y))/((sum(x))^2 - n*xSq);
slope = (sum(x)*sum(y) - n*xy)/((sum(x))^2 - n*xSq);

% Output calculated slope and intercept
myReg = [slope,intercept];

end