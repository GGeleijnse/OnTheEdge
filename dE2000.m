function [dE] = dE2000(RGB1,RGB2)
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

Lab1 = rgb2lab(RGB1);
Lab2 = rgb2lab(RGB2);

L1    = Lab1(:,1);
L2    = Lab2(:,1);
L_bar = (L1 + L2)/2;
dL    = L2 - L1;

a1_s = Lab1(:,2);
a2_s = Lab2(:,2);

b1  = Lab1(:,3);
b2  = Lab2(:,3);

C1_s = (a1_s.^2 + b1.^2).^(0.5);
C2_s = (a2_s.^2 + b2.^2).^(0.5);
C_sbar = (C1_s + C2_s)/2;

G = 0.5 * ( 1-((C_sbar.^7./(C_sbar.^7+25^7)).^(0.5)));

a1 = a1_s.*(1+G);
a2 = a2_s.*(1+G);

C1    = (a1.^2 + b1.^2).^(0.5);
C2    = (a2.^2 + b2.^2).^(0.5);
C_bar = (C1 + C2)/2;
dC    = C2 - C1;

h1    = atan2d(b1,a1);
idcs = find(h1 < 0);
h1(idcs) = h1(idcs)+360;

h2    = atan2d(b2,a2);
idcs  = find(h2 < 0);
h2(idcs) = h2(idcs)+360;

h_bar = (h1 + h2) /2;
idcs  = find(abs(h1 - h2) > 180);
h_bar(idcs) = h_bar(idcs) + 180;

dh       = h2 - h1;

idcs     = find(abs(h1 - h2) > 180 & h2 <= h1);
dh(idcs) = h2(idcs) - h1(idcs) + 360;

idcs     = find(abs(h1 - h2) > 180 & h2 > h1);
dh(idcs) = h2(idcs) - h1(idcs) - 360;

dH =2*(C1.*C2).^0.5.*sind(dh/2);

T = 1- 0.17*cosd(h_bar-30)+0.24*cosd(2*h_bar)+0.32*cosd(3*h_bar+6)-0.20*cosd(4*h_bar-63);

S_l = 1 + (0.015*(L_bar-50).^2)./((20+(L_bar-50).^2).^(0.5));
S_c = 1+0.045*C_bar;
S_h = 1+0.015*C_bar.*T;

k_l = 1;
k_c = 1;
k_h = 1;

R_c = 2*(C_bar.^7./(C_bar.^7+25^7)).^(0.5);
dtheta = 30*exp(-((h_bar-275)/25).^2);
R_T = -sind(2*dtheta).*R_c;

dE = sqrt( (dL./(k_l*S_l)).^2 + (dC./(k_c*S_c)).^2 +(dH./(k_h*S_h)).^2 + ...
           (R_T.*(dC./(k_c*S_c)).*(dH./(k_h*S_h))));
       %%
       
end

