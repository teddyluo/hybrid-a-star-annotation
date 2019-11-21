%DUBINS_CURVE   Find the Dubins path (shortest curve) between two points.
%   PATH = DUBINS_CURVE(P1, P2, r, stepsize) finds the shortest curve that
%   connects two points in the Euclidean plane with a constraint of the
%   curvature of the path. The start and finish orientations P1 and P2 are
%   defined as [x, y, theta]. The turning radius (r) and stepsize will be
%   defined automatically if their value is <= 0. The output PATH is an [mx3]
%   array consisting of m rows of [x, y, theta] values.  
%
%   PATH = DUBINS_CURVE(P1, P2, r, stepsize, quiet) performs the same as above,
%   however if quiet == true, then no plots of command window output will be
%   generated. Ommitting this input will result in quiet = false/0.
%
%   This function handles the interface to dubins_core.m to give a more
%   intuitive tool for finding the Dubins path. 

% Reference:
%       https://github.com/AndrewWalker/Dubins-Curves#shkel01
%       Shkel, A. M. and Lumelsky, V. (2001). "Classification of the Dubins
%                  set". Robotics and Autonomous Systems 34 (2001) 179�V202
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Original Source: Andrew Walker
% MATLAB-lization: Ewing Kang
% Date: 2016.2.28
% contact: f039281310 [at] yahoo.com.tw
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2016, Ewing Kang                                                 % 
%                                                                                %
% Permission is hereby granted, free of charge, to any person obtaining a copy   %
% of this software and associated documentation files (the "Software"), to deal  %
% in the Software without restriction, including without limitation the rights   %
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell      %
% copies of the Software, and to permit persons to whom the Software is          %  
% furnished to do so, subject to the following conditions:                       %
%                                                                                %
% The above copyright notice and this permission notice shall be included in     %
% all copies or substantial portions of the Software.                            %
%                                                                                %
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR     %
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,       %
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE    %
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER         %
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,  %
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN      %
% THE SOFTWARE.                                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function path = dubins_curve(p1, p2, r, stepsize, quiet)
    
    %%%%%%%%%%%%%%%%%%%%%%%%% DEFINE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % there are 6 types of dubin's curve, only one will have minimum cost
    % LSL = 1;
	% LSR = 2;
	% RSL = 3;
	% RSR = 4;
	% RLR = 5;
    % LRL = 6;
    
    % The three segment types a path can be made up of
    % L_SEG = 1;
    % S_SEG = 2;
    % R_SEG = 3;

    % The segment types for each of the Path types
    %{
    DIRDATA = [ L_SEG, S_SEG, L_SEG ;...
                L_SEG, S_SEG, R_SEG ;...
                R_SEG, S_SEG, L_SEG ;...
                R_SEG, S_SEG, R_SEG ;...
                R_SEG, L_SEG, R_SEG ;...
                L_SEG, R_SEG, L_SEG ]; 
    %}
            
    % the return parameter from dubins_core
    % param.p_init = p1;              % the initial configuration
    % param.seg_param = [0, 0, 0];    % the lengths of the three segments
    % param.r = r;                    % model forward velocity / model angular velocity turning radius
    % param.type = -1;                % path type. one of LSL, LSR, ... 
    %%%%%%%%%%%%%%%%%%%%%%%%% END DEFINE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Handle inputs.
    if nargin < 3
        error('Function requires at least two inputs.');
    elseif nargin < 4
        stepsize = 0;
    elseif nargin < 5 
        quiet = 0;  %Default/undefined is not quiet
    end
    
    if ~quiet
        close(findobj('type','figure','name','Dubins curve'));
        tic;
    end
    
    % Check if the destination lies on the circle that can be reach
    % directly from the starting point
    if(p1(3)~=p2(3))
        T = [cos(p1(3)), sin(p1(3));...
             cos(p2(3)), sin(p2(3)) ];
        Y = [p1(1)*cos(p1(3)) + p1(2)*sin(p1(3)); ...
             p2(1)*cos(p2(3)) + p2(2)*sin(p2(3)) ];
        X = T \ Y;
        if( norm(X-reshape(p1(1:2), [2,1]),2) == r ) && ( norm(X-reshape(p2(1:2),[2,1]),2) == r )
            warning('p2 lies on the turning circle from the p2, dubins curve may be suboptimal');
        end
    end
        
    
    % main function
    param = dubins_core(p1, p2, r);
    if stepsize <= 0
        stepsize = dubins_length(param)/1000;
    end
    path = dubins_path_sample_many(param, stepsize);
    
    % plot if not quiet
    if ~quiet
        disp('dubins calculation time'); toc;
        % plotting
        tic;    % most of the time is spent on plotting
        figure('name','Dubins curve');
        plot(path(:,1), path(:,2)); axis equal; hold on
        scatter(p1(1), p1(2), 45, '*','r','LineWidth',1); hold on;
        scatter(p2(1), p2(2), 45, 'square','b','LineWidth',1); hold on;
        text(p1(1), p1(2),'start','HorizontalAlignment','center');
        text(p2(1), p2(2),'end','VerticalAlignment','top');
        disp('plot drawing time'); toc;
    end
end

function path = dubins_path_sample_many( param, stepsize)
    if param.flag < 0
        path = 0;
        return
    end
    length = dubins_length(param);
    path = -1 * ones(floor(length/stepsize), 3);
    x = 0;
    i = 1;
    while x <= length
        path(i, :) = dubins_path_sample( param, x );
        x = x + stepsize;
        i = i + 1;
    end
    return
end

function length = dubins_length(param)
    length = param.seg_param(1);
    length = length + param.seg_param(2);
    length = length + param.seg_param(3);
    length = length * param.r;
end


%{
 * Calculate the configuration along the path, using the parameter t
 *
 * @param path - an initialised path
 * @param t    - a length measure, where 0 <= t < dubins_path_length(path)
 * @param q    - the configuration result
 * @returns    - -1 if 't' is not in the correct range
%}
function end_pt = dubins_path_sample(param, t)
    if( t < 0 || t >= dubins_length(param) || param.flag < 0)
        end_pt = -1;
        return;
    end

    % tprime is the normalised variant of the parameter t
    tprime = t / param.r;

    % In order to take rho != 1 into account this function needs to be more complex
    % than it would be otherwise. The transformation is done in five stages.
    %
    % 1. translate the components of the initial configuration to the origin
    % 2. generate the target configuration
    % 3. transform the target configuration
    %      scale the target configuration
    %      translate the target configration back to the original starting point
    %      normalise the target configurations angular component

    % The translated initial configuration
    p_init = [0, 0, param.p_init(3) ];
    
    %%%%%%%%%%%%%%%%%%%%%%%%% DEFINE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % The three segment types a path can be made up of
    L_SEG = 1;
    S_SEG = 2;
    R_SEG = 3;

    % The segment types for each of the Path types
    DIRDATA = [ L_SEG, S_SEG, L_SEG ;...
                L_SEG, S_SEG, R_SEG ;...
                R_SEG, S_SEG, L_SEG ;...
                R_SEG, S_SEG, R_SEG ;...
                R_SEG, L_SEG, R_SEG ;...
                L_SEG, R_SEG, L_SEG ]; 
    %%%%%%%%%%%%%%%%%%%%%%%%% END DEFINE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Generate the target configuration
    types = DIRDATA(param.type, :);
    param1 = param.seg_param(1);
    param2 = param.seg_param(2);
    mid_pt1 = dubins_segment( param1, p_init, types(1) );
    mid_pt2 = dubins_segment( param2, mid_pt1,  types(2) );
    
    % Actual calculation of the position of tprime within the curve
    if( tprime < param1 ) 
        end_pt = dubins_segment( tprime, p_init,  types(1) );
    elseif( tprime < (param1+param2) ) 
        end_pt = dubins_segment( tprime-param1, mid_pt1,  types(2) );
    else 
        end_pt = dubins_segment( tprime-param1-param2, mid_pt2,  types(3) );
    end

    % scale the target configuration, translate back to the original starting point
    end_pt(1) = end_pt(1) * param.r + param.p_init(1);
    end_pt(2) = end_pt(2) * param.r + param.p_init(2);
    end_pt(3) = mod(end_pt(3), 2*pi);
    return;
end

%{
 returns the parameter of certain location according to an inititalpoint,
 segment type, and its corresponding parameter
%}
function seg_end = dubins_segment(seg_param, seg_init, seg_type)
    L_SEG = 1;
    S_SEG = 2;
    R_SEG = 3;
    if( seg_type == L_SEG ) 
        seg_end(1) = seg_init(1) + sin(seg_init(3)+seg_param) - sin(seg_init(3));
        seg_end(2) = seg_init(2) - cos(seg_init(3)+seg_param) + cos(seg_init(3));
        seg_end(3) = seg_init(3) + seg_param;
    elseif( seg_type == R_SEG )
        seg_end(1) = seg_init(1) - sin(seg_init(3)-seg_param) + sin(seg_init(3));
        seg_end(2) = seg_init(2) + cos(seg_init(3)-seg_param) - cos(seg_init(3));
        seg_end(3) = seg_init(3) - seg_param;
    elseif( seg_type == S_SEG ) 
        seg_end(1) = seg_init(1) + cos(seg_init(3)) * seg_param;
        seg_end(2) = seg_init(2) + sin(seg_init(3)) * seg_param;
        seg_end(3) = seg_init(3);
    end
end
