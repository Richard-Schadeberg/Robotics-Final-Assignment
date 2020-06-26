classdef PumaClass < handle
properties
    p560
    q = [0 128 -11.7 13.8 66 0] * pi/180; % starting pose chosen by teach
    qMin % taken from robot properties
    qMax %
    hasTool = false;
    tool % Prop object created by GiveTool
    toolTr % Relative Tr from effector to tool
    qVelMax =   [ 8 10 10  5  5  5]/30; % converted from m/s to m/step
    qAccMax =   [10 12 12  8  8  8]/(30^2);
    qAccDot =   [30 40 40 20 20 20]/(30^3);
    eVelMax =   [ 5  5  5  1  1  1]/1000; % arbitrarily chosen to avoid robot moving too fast to follow
    torqueMax = [97.6 186.4 89.4 24.2 20.1 21.3];
end
    methods
function self = PumaClass(baseTr)
    mdl_puma560;
    self.p560 = p560; % create SerialLink robot
    self.p560.base = baseTr; % move to given Tr
    self.qMin = self.p560.qlim(:,1)'; % horizontal vector
    self.qMax = self.p560.qlim(:,2)';
    self.p560.plot(self.q);
    [x y z] = transl(baseTr); % set plot limits to 2m cube around base
    xlim([x-1,x+1])
    ylim([y-1,y+1])
    zlim([z-1,z+1])
end
function giveTool(self,toolName,toolTr,toolMass)
    self.tool = Prop(toolName,self.p560.fkine(self.q) * toolTr);
    self.toolTr = toolTr;
    self.hasTool = true;
    self.p560.payload(toolMass,transl(toolTr)); % serialLink kinematics
end
function animateCartesian(self,goalTr) % Cartesian Space
    self.animateQMatrix(self.getCartesianQMatrix(goalTr));
end 
function animateJoints(self,goalTr)    % Joint Space
    goalJoints = self.p560.ikcon(goalTr,self.q);
    qMatrix = self.getJointsQMatrix(goalJoints);
    self.animateQMatrix(qMatrix);
end

function animateQMatrix(self,qMatrix)  % qMatrix is a list of joint angles to step through
    len = size(qMatrix); % can't use length here because sometimes qMatrix is 1x6, and in that case len should be 1, not 6
    len = len(1);
    for i = 1:len
        if self.hasTool
            self.tool.updatePos(self.p560.fkine(qMatrix(i,:)) * self.toolTr) % move tool to reflect new pose
        end
        animate(self.p560,qMatrix(i,:));
        drawnow();
    end
    self.q = qMatrix(i,:);
end
function qMatrix = getCartesianQMatrix(self, goalTransform)
    q = self.q;
    pq = q;
    for i=1:300 % assume any cartesian motion that takes more than 300 steps is an error
        % find desired end-effector velocities as the velocities required
        % to get to the goal in 1 step
        currentTransform = self.p560.fkine(q);
        coords = transl(goalTransform - currentTransform);
        rDiff3x3 = (tr2rt(goalTransform) * (tr2rt(currentTransform)') - eye(3));
        endVelocities = [coords ; rDiff3x3(3,2) ; rDiff3x3(1,3) ; rDiff3x3(2,1)]; % vertical vector
        % clamp these velocities down to a reasonable speed
        endVelocities = clampVectorScale(endVelocities', self.eVelMax)'; % vertical vector
        % convert to joint space and clamp to robot's limits
        qVelocities = inv(self.p560.jacob0(q)) * endVelocities; % vertical vector
        qVelocities = clampVectorScale(qVelocities', self.qVelMax); % horizontal vector
        % nq is the proposed next step for the robot
        % pq -> q -> nq
        nq = q + qVelocities;
        nq = self.clampForDynamics(pq,q,nq); % account for torque limits
        % it's possible this last clamping would cause the robot to exceed
        % its torque limits, if it's moving very fast with a large payload
        nq = self.clampForJointLimits(nq);
        qMatrix(i,:) = nq;
        pq = q; % shift all 'q's back so we can create a new nq next loop
        q = nq;
        if max(abs(endVelocities)) < 0.01 % if we're close to the destination
            break;
        end
%         animate(self.p560,q);
%         drawnow();
    end
end
function qMatrix = getJointsQMatrix(self, goalJoints)
    maxLen = 1;
    for j = 1:6 % find how many steps it takes to get all joints to the goal pose
        distance = abs(goalJoints(j) - self.q(j));
        % curves{j} is a curve from 0->1 that accelerates up to qVelMax,
        % maintains it, then decelerates again at the end
        curves{j} = improvedCurveGen(self.qVelMax(j)/distance,self.qAccMax(j)/distance);
        if length(curves{j}) > maxLen
            maxLen = length(curves{j});
        end
    end
    for j = 1:6
        curve = [curves{j} ones(1,maxLen-length(curves{j}))];    % pad curve to max length with 1s
        curve = curve * (goalJoints(j) - self.q(j)) + self.q(j); % convert curve from 0->1 to start->goal
        qMatrix(:,j) = curve';
    end 
end
function nq = clampForDynamics(self,pq,q,nq) % pq - previous q, nq - next q
    dt = 1/30;                          % robot is running at 30 fps
    qd = q - pq;                        % speed from previous step to current
    qdd = (1/dt)^2 * (nq - q - dt*qd);  % proposed acceleration by nq
    B = self.p560.pay([0 0 215 0 0 0]',self.p560.jacob0(q)); % torque from 215N blast stream
    M = self.p560.inertia(q);
    C = self.p560.coriolis(q,qd);
    g = self.p560.gravload(q);
    tau = (M*qdd' + C*qd' + g')' + B;   % find torque on joints for proposed nq
    for j = 1:6                         % reduce torque on joints to be within limits
        if abs(tau(j)) > self.torqueMax(j)
            tau(j) = sign(tau(j)) * self.torqueMax(j);
        end
    end
    qdd = (inv(M)*((tau - B)' - C*qd' - g'))'; % rearrange torque formula to solve for acceleration
    nq = q + dt*qd + dt^2*qdd;                 % update qn to reflect reduced acceleration
end
function  q = clampForJointLimits(self,q)
    for i=1:6
        if q(i) > self.qMax(i)
            q(i) = self.qMax(i);
        elseif q(i) < self.qMin(i)
            q(i) = self.qMin(i);
        end
    end
end
    end
end
function curve = improvedCurveGen(maxVel,acc) % generates a curve of numbers from 0 - 1
    t = 1;
    curve(1) = 0;
    while curve(t) <= 0.5 % generate curve up to halfway
        t = t + 1;
        if maxVel > t * acc % parabola accelerating up to maxVel
            curve(t) = 0.5 * t^2 * acc;
        else % straight line once curve reaches maxVel
            curve(t) = (0.5 * maxVel^2 / acc) + (maxVel * (t - maxVel/acc));
        end
    end
    % curve is symmetrical, so flip it for second half
    t = t - 1; % trim last value as it's > 0.5
    curve = [curve(1:t) flip(1 - curve(1:t-1))];
end
function vector = clampVectorScale(input,limits) % only works on horizontal vectors
    % scales down all values in input so that none have a higher magnitude
    % than their counterpart in limits. Maintains same ratio of values
    vectorRatio = max(abs(input/limits));
    if vectorRatio > 1
        vector = input / vectorRatio;
    else
        vector = input;
    end
end