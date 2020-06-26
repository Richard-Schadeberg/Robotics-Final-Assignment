classdef Prop
	properties
		faces;
		points;
		data;
		numPoints;
		prop_h;
    end
	methods
		function self = Prop(name,startTr)
			[self.faces, self.points, self.data] = plyread(name+".ply", "tri");
			numPoints = size(self.points);
			self.numPoints = numPoints(1);
			self.prop_h = initProp(self);
            self.updatePos(startTr);
            drawnow();
        end
		function updatePos(self, propTr) % move the prop to a given Tr
			for j = 1:self.numPoints
				self.prop_h.Vertices(j,:) = transl(propTr*transl(self.points(j,:)))';
			end
        end
	end
end
function prop_h = initProp(self) % plot points, faces, and colour data
    hold on
    prop_h=trisurf(self.faces,self.points(:,1),self.points(:,2),self.points(:,3),"LineStyle","none");
    hold off
    prop_h.FaceVertexCData = [self.data.vertex.red self.data.vertex.green self.data.vertex.blue]/255;
    prop_h.FaceColor = 'interp';
end
