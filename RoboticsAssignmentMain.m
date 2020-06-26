% --- constants ---
student_number = 11980935;
toolWidth = 0.01; % 10 mm
toolMass = 2.1; % kg

puma_base = transl(1.198,0.935,1  ); % absolute
drumTr =    transl(0.4,  0.4,  0.1); % absolute
robotToBarrelTr =    HomInvert(puma_base) * drumTr;   % relative

effToStreamTr =  transl(0,0,0.2) * trotx(pi/4); % relative
streamToEffTr =  HomInvert(effToStreamTr);       % relative

windowBottomLeft =  transl(0.20296, 0.52539, 0.59755); % relative
windowBottomRight = transl(0.39216, 0.52539, 0.59755); % relative
windowTopLeft =     transl(0.20296, 0.67459, 0.59755); % relative
windowTopRight =    transl(0.39216, 0.67459, 0.59755); % relative
toolHeight =        transl(0,0,0.2); % relative

% --- main ---
clf;
puma = PumaClass(puma_base);
puma.giveTool('tool',transl(0,0,0),toolMass);
drum = Prop('drum',drumTr);

cleanBarrelWindow(puma,windowBottomLeft,windowBottomRight,windowTopLeft,windowTopRight,toolWidth,toolHeight,streamToEffTr,drumTr)

function cleanBarrelWindow(puma,windowBottomLeft,windowBottomRight,windowTopLeft,windowTopRight,toolWidth,toolHeight,streamToEffTr,drumTr)
    % move so the tool points at the top left corner of the window
    goalTr = drumTr * windowTopLeft * toolHeight * trotx(pi) * streamToEffTr;
    puma.animateJoints(goalTr);
    % generate list of waypoints that zig-zags down window. These are relative to the drum and have no orientation:
    waypoints = generateWaypoints(windowBottomLeft,windowBottomRight,windowTopLeft,windowTopRight,toolWidth);
    for i=1:length(waypoints)
        % position that points tool at waypoint:
        goalTr = drumTr * transl(waypoints(i,:)') * toolHeight * trotx(pi) * streamToEffTr;
        puma.animateCartesian(goalTr);
    end
end

function waypoints = generateWaypoints(bottomLeft,bottomRight,topLeft,topRight,toolWidth)
    top2BottomTr = HomInvert(topLeft) * bottomLeft;
    dist = distance(transl(top2BottomTr),[0 0 0]');
    i = 1;
    d = 0;
    while true;
        shift = transl(transl(top2BottomTr) * d / dist);
        waypoints(i,:) = transl(topLeft * shift)';
        i = i + 1;
        waypoints(i,:) = transl(topRight * shift)';
        i = i + 1;
        if d > dist
            break;
        end
        d = d + toolWidth;
        shift = transl(transl(top2BottomTr) * d / dist);
        waypoints(i,:) = transl(topRight * shift)';
        i = i + 1;
        waypoints(i,:) = transl(topLeft * shift)';
        i = i + 1;
        if d > dist
            break;
        end
        d = d + toolWidth;
    end
end