%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function will find the shortest dubins curve between two points
% Input: 
%   p1/p2: Initial and ending 2-D pose
%          In row vectors, e.g. [x, y, theta]
%   r: turning radius of the curve
% Output: 
%   param: a struct that includes 4 field:
%     p_init: Initial pose, equals to input p1
%     type: One of the 6 types of the dubins curve
%     r: Turning radius, same as input r, also the scaling factor for 
%        the dubins paramaters
%     seg_param: angle or normalized length, in row vector [pr1, pr2, pr3]
% Reference:
% 	https://github.com/AndrewWalker/Dubins-Curves#shkel01
%   Shkel, A. M. and Lumelsky, V. (2001). "Classification of the Dubins
%                 set". Robotics and Autonomous Systems 34 (2001) 179¡V202
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Original Source (in C): Andrew Walker
% MATLAB-lization: Ewing Kang
% Date: 2016.2.28
% contact: f039281310 [at] yahoo.com.tw
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2018 Ewing Kang                                           %
% Released under GPLv3 license                                            %
% This function is a MATLAB re-written from Andrew Walker's work, which   %
% was originally distributed under MIT license in C language              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function param = dubins_core(p1, p2, r)
    %%%%%%%%%%%%%%%%%% DEFINE %%%%%%%%%%%%%%%%%
    % Here are some usefuldefine headers for better implementation
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
    %%%%%%%%%%%%%%%% END DEFINE %%%%%%%%%%%%%%%%
            
    % the return parameter
    param.p_init = p1;              % the initial configuration
    param.seg_param = [0, 0, 0];    % the lengths of the three segments
    param.r = r;                    % model forward velocity / model angular velocity turning radius
    param.type = -1;                % path type. one of LSL, LSR, ... 
    param.flag = 0;
    
    %%%%%%%%%%%%%%%%%%%%%%%%% START %%%%%%%%%%%%%%%%%%%%%%%%%
    % First, basic properties and normalization of the problem
    dx = p2(1) - p1(1);
    dy = p2(2) - p1(2);
    D = sqrt( dx^2 + dy^2 );
    d = D / r;                  % distance is shrunk by r, this make lengh calculation very easy
    if( r <= 0 )
        param.flag = -1;
        return;
    end
    theta = mod(atan2( dy, dx ), 2*pi);
    alpha = mod((p1(3) - theta), 2*pi);
    beta  = mod((p2(3) - theta), 2*pi);

    % Second, we find all possible curves
    best_word = -1;
    best_cost = -1;
    test_param(1,:) = dubins_LSL(alpha, beta, d);
    test_param(2,:) = dubins_LSR(alpha, beta, d);
    test_param(3,:) = dubins_RSL(alpha, beta, d);
    test_param(4,:) = dubins_RSR(alpha, beta, d);
    test_param(5,:) = dubins_RLR(alpha, beta, d);
    test_param(6,:) = dubins_LRL(alpha, beta, d);
    
    for i = 1:1:6
        if(test_param(i,1) ~= -1) 
            cost = sum(test_param(i,:));
            if(cost < best_cost) || (best_cost == -1)
                best_word = i;
                best_cost = cost;
                param.seg_param = test_param(i,:);
                param.type = i;
            end
        end
    end

    if(best_word == -1) 
        param.flag = -2;             % NO PATH
        return;
    else
        return;
    end
end

function param = dubins_LSL(alpha, beta, d)
    tmp0 = d + sin(alpha) - sin(beta);
    p_squared = 2 + (d*d) -(2*cos(alpha - beta)) + (2*d*(sin(alpha) - sin(beta)));
    if( p_squared < 0 )
        param = [-1, -1, -1];
        return;
    else
        tmp1 = atan2( (cos(beta)-cos(alpha)), tmp0 );
        t = mod((-alpha + tmp1 ), 2*pi);
        p = sqrt( p_squared );
        q = mod((beta - tmp1 ), 2*pi);
        param(1) = t; 
        param(2) = p; 
        param(3) = q;
        return ;
    end
end
function param = dubins_LSR(alpha, beta, d)
    p_squared = -2 + (d*d) + (2*cos(alpha - beta)) + (2*d*(sin(alpha)+sin(beta)));
    if( p_squared < 0 )
        param = [-1, -1, -1];
        return;
    else
        p    = sqrt( p_squared );
        tmp2 = atan2( (-cos(alpha)-cos(beta)), (d+sin(alpha)+sin(beta)) ) - atan2(-2.0, p);
        t    = mod((-alpha + tmp2), 2*pi);
        q    = mod(( -mod((beta), 2*pi) + tmp2 ), 2*pi);
        param(1) = t; 
        param(2) = p; 
        param(3) = q;
        return ;
    end
end
function param = dubins_RSL(alpha, beta, d)
    p_squared = (d*d) -2 + (2*cos(alpha - beta)) - (2*d*(sin(alpha)+sin(beta)));
    if( p_squared< 0 ) 
        param = [-1, -1, -1];
        return;
    else
        p    = sqrt( p_squared );
        tmp2 = atan2( (cos(alpha)+cos(beta)), (d-sin(alpha)-sin(beta)) ) - atan2(2.0, p);
        t    = mod((alpha - tmp2), 2*pi);
        q    = mod((beta - tmp2), 2*pi);
        param(1) = t;
        param(2) = p; 
        param(3) = q;
        return ;
    end
end
function param = dubins_RSR(alpha, beta, d)
    tmp0 = d-sin(alpha)+sin(beta);
    p_squared = 2 + (d*d) -(2*cos(alpha - beta)) + (2*d*(sin(beta)-sin(alpha)));
    if( p_squared < 0 )
        param = [-1, -1, -1];
        return;
    else
        tmp1 = atan2( (cos(alpha)-cos(beta)), tmp0 );
        t = mod(( alpha - tmp1 ), 2*pi);
        p = sqrt( p_squared );
        q = mod(( -beta + tmp1 ), 2*pi);
        param(1) = t; 
        param(2) = p; 
        param(3) = q;
        return;
    end
end
function param = dubins_RLR(alpha, beta, d)
    tmp_rlr = (6. - d*d + 2*cos(alpha - beta) + 2*d*(sin(alpha)-sin(beta))) / 8.;
    if( abs(tmp_rlr) > 1)
        param = [-1, -1, -1];
        return;
    else
        p = mod(( 2*pi - acos( tmp_rlr ) ), 2*pi);
        t = mod((alpha - atan2( cos(alpha)-cos(beta), d-sin(alpha)+sin(beta) ) + mod(p/2, 2*pi)), 2*pi);
        q = mod((alpha - beta - t + mod(p, 2*pi)), 2*pi);
        param(1) = t;
        param(2) = p;
        param(3) = q;
        
        return;
    end
end
function param = dubins_LRL(alpha, beta, d)
    tmp_lrl = (6. - d*d + 2*cos(alpha - beta) + 2*d*(- sin(alpha) + sin(beta))) / 8.;
    if( abs(tmp_lrl) > 1)
        param = [-1, -1, -1]; return;
    else
        p = mod(( 2*pi - acos( tmp_lrl ) ), 2*pi);
        t = mod((-alpha - atan2( cos(alpha)-cos(beta), d+sin(alpha)-sin(beta) ) + p/2), 2*pi);
        q = mod((mod(beta, 2*pi) - alpha -t + mod(p, 2*pi)), 2*pi);
        param(1) = t;
        param(2) = p;
        param(3) = q;
        return;
    end
end