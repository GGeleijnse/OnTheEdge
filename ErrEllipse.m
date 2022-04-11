function [avg_x,avg_y,r_ellipse] = ErrEllipse(X,Y)
%%
% Copyright (c) 2021, Erasmus University Medical Center Rotterdam 
% (?Erasmus MC?). All rights reserved. 
% 
% Non-commercial use of the MATLAB program for research purposes is 
% permitted provided that the following conditions are met:
% 
% ?	The use of the MATLAB program must be duly acknowledged and all 
% advertising materials mentioning features or use of the MATLAB program
% must display the following acknowledgement: 
% Erasmus MC. G. Geleijnse et al 2021. "Measuring Image Quality of ENT 
% Chip-on-tip Endoscopes, Journal of Imaging Science and Technology, 
% vol. 65, no. 2, 2021.?.
% ?	Redistributions of source code must retain the above copyright notice,
% this list of conditions and the following disclaimer.
% ?	Any change made to the original MATLAB program needs to be clearly 
% indicated in the header of the source-code and accompanied documents; 
% ?	Changes and improvements need to be shared with the Erasmus MC, 
% att. Geert Geleijnse; g.geleijnse@erasmusmc.nl;
% ?	It is understood and agreed that any change or improvement made to the
% original MATLAB program will be owned by Erasmus MC as being part of the
% original work, unless the improvement can run independently from (and
% without the use of) the original program and needs to be considered as a 
% new ?work? according to the Dutch Copyright Act (Dutch: ?Auteurswet?);
% ?	Neither the name of Erasmus MC nor the names of its contributors may be
% used to endorse or promote products derived from this software without
% specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY ERASMUS MC ?AS IS? AND ANY EXPRESS OR 
% IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
% OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
% IN NO EVENT SHALL ERASMUS MC BE LIABLE FOR ANY DIRECT, INDIRECT,
% INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
% NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
% DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
% THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF 
% THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
% 
% For the avoidance of doubt, commercial use of any kind (including but not
% limited to direct and indirect exploitation, validation of commercial
% products etc.) is prohibited without approval of Erasmus MC and requires 
% a commercial license to be negotiated by Erasmus MC?s Technology Transfer 
% Office; tto@erasmusmc.nl.
% 
% If you are contacted by any party interested in the use of the MATLAB
% program, please refer them to Erasmus MC, Department of Ear, Nose, Throat
% (ENT), Geert Geleijnse: g.geleijnse@erasmusmc.nl.




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

