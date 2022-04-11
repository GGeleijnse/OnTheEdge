function [dE] = dE2000(RGB1,RGB2)
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

