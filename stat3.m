function [arg_avg, arg_std] = stat3(arg1)
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


    % convert to gray values according to Rec. ITU-R BT.709-6 in
    % ISO15739_2017
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


