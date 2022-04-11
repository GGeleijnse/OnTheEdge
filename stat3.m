function [arg_avg, arg_std] = stat3(arg1)
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


    %convert to gray values according to Rec. ITU-R BT.709-6 in
    %ISO15739_2017
    arg1  = double(arg1);
    R = reshape(arg1(:,:,1),1,[]);
    G = reshape(arg1(:,:,2),1,[]);
    B = reshape(arg1(:,:,3),1,[]);
    
    Y  = 0.2126*R + 0.7152*G + 0.0722*B;
    
    
    arg_avg   = mean(Y);
    Y_std     = std(Y);
    RmY_std   = std(R-Y);
    BmY_std   = std(B-Y);
    %compute camera noise with chrominance noise 
    %ISO15739_2017
    arg_std   = sqrt(Y_std^2 + 0.279*RmY_std^2 + 0.088*BmY_std^2);
end


