function [avg_x,avg_y,r_ellipse] = ErrEllipse(X,Y)
%%
% Copyright 2021 Erasmus University Medical Center Rotterdam, Geert Geleijnse
% 
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% 
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
%%



% Calculate the eigenvectors and eigenvalues
covariance = cov(X,Y);
[eigenvec, eigenval ] = eig(covariance);

% Get the index of the largest eigenvector
[largest_eigenvec_ind_c, r] = find(eigenval == max(max(eigenval)));
largest_eigenvec = eigenvec(:, largest_eigenvec_ind_c);

% Get the largest eigenvalue
largest_eigenval = max(max(eigenval));

% Get the smallest eigenvector and eigenvalue
if(largest_eigenvec_ind_c == 1)
    smallest_eigenval = max(eigenval(:,2));
    smallest_eigenvec = eigenvec(:,2);
else
    smallest_eigenval = max(eigenval(:,1));
    smallest_eigenvec = eigenvec(1,:);
end

% Calculate the angle between the x-axis and the largest eigenvector
angle = atan2(largest_eigenvec(2), largest_eigenvec(1));

% This angle is between -pi and pi.
% Let's shift it such that the angle is between 0 and 2pi

if(angle < 0)
    angle = angle + 2*pi;
end

% Get the coordinates of the data mean
avg_x = mean(mean(X));
avg_y = mean(mean(Y));

% Get the 95% confidence interval error ellipse
chisquare_val = 2.4477; %sqrt(chi2inv( ))
theta_grid = linspace(0,2*pi);

a=chisquare_val*sqrt(largest_eigenval);
b=chisquare_val*sqrt(smallest_eigenval);

% the ellipse in x and y coordinates 
ellipse_x_r  = a*cos( theta_grid );
ellipse_y_r  = b*sin( theta_grid );

%Define a rotation matrix
R = [ cos(angle) sin(angle); -sin(angle) cos(angle) ];

%let's rotate the ellipse to some angle phi
r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;

end

