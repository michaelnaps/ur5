function [links, joints] = load_urdf(urdf_file)
    % This function parse the ROS URDF file
    %
    % @note At this moment, this parser only support the 'link' and 'joint'
    % elements in the URDF file. 
    %
    % Return values:
    % links: an array of rigid links structure @type structure
    % joints an array of rigid joints structure @type structure
    
    
    
    urdf = xmlread(urdf_file);
    
    xml_robot = urdf.getElementsByTagName('robot').item(0);
    assert(~isempty(xml_robot),['The provided URDF file does not contains ',...
        'the proper robot element. Please provide a valid URDF file.']);
    
    
    
    
    % extract rigid links
    xml_links = xml_robot.getElementsByTagName('link');
    
    Rz = @(th) [cos(th), -sin(th), 0; sin(th), cos(th), 0; 0,0,1];
    Ry = @(th) [cos(th), 0, sin(th); 0, 1, 0; -sin(th), 0, cos(th)];
    Rx = @(th) [1,0,0; 0, cos(th), -sin(th); 0, sin(th), cos(th)];
    Rot = @(q) Rz(q(3))*Ry(q(2))*Rx(q(1));
    
    num_links = xml_links.getLength();
    links = struct();
    index = 1;
    for i=0:num_links-1
        xml_link = xml_links.item(i);
        if xml_link.hasChildNodes
            
            links(index).Name = char(xml_link.getAttribute('name'));
            inertial = xml_link.getElementsByTagName('inertial').item(0);
            origin = inertial.getElementsByTagName('origin').item(0);
            mass   = inertial.getElementsByTagName('mass').item(0);
            inertia   = inertial.getElementsByTagName('inertia').item(0);
            
            
            ixx = str2double(inertia.getAttribute('ixx'));
            ixy = str2double(inertia.getAttribute('ixy'));
            ixz = str2double(inertia.getAttribute('ixz'));
            iyy = str2double(inertia.getAttribute('iyy'));
            iyz = str2double(inertia.getAttribute('iyz'));
            izz = str2double(inertia.getAttribute('izz'));
            
            links(index).Mass = str2double(mass.getAttribute('value'));
            %%% To handle the case where inertial frame is the same as body
            % frame. (in URDF, this is normally ignored)
            try
               links(index).Offset = str2num(origin.getAttribute('xyz')); %#ok<*ST2NM>
               rpy = str2num(origin.getAttribute('rpy'));
            catch
               links(index).Offset = zeros(1,3);
               rpy = zeros(1,3);
            end
           
            if isempty(rpy)
                links(index).R = Rot(zeros(1,3));
            else
                links(index).R = Rot(rpy);
            end
            links(index).Inertia = [ixx,ixy,ixz; ixy,iyy,iyz; ixz, iyz, izz];
            
            index = index + 1;
        end
        
    end
    
    
    
    % extract joints
    xml_joints = xml_robot.getElementsByTagName('joint');    
    num_joints = xml_joints.getLength();
    joints = struct();
    index = 1;
    for i=0:num_joints-1
        xml_joint = xml_joints.item(i);
        
        if xml_joint.hasChildNodes
            if isempty(char(xml_joint.getAttribute('type')))
                continue;
            end
            joints(index).Name = char(xml_joint.getAttribute('name'));
            joints(index).Type = char(xml_joint.getAttribute('type'));
            
            origin = xml_joint.getElementsByTagName('origin').item(0);
            axis = xml_joint.getElementsByTagName('axis').item(0);
            parent = xml_joint.getElementsByTagName('parent').item(0);
            child = xml_joint.getElementsByTagName('child').item(0);
            
            
            joints(index).Offset = str2num(origin.getAttribute('xyz'));
            rpy = str2num(origin.getAttribute('rpy'));
            if isempty(rpy)
                joints(index).R = Rot(zeros(1,3));
            else
                joints(index).R = Rot(rpy);
            end
            joints(index).Parent = char(parent.getAttribute('link'));
            joints(index).Child  = char(child.getAttribute('link'));
            
            if ~strcmp(joints(index).Type, 'fixed')
                joints(index).Axis = str2num(axis.getAttribute('xyz'));                
            end
            limit = xml_joint.getElementsByTagName('limit').item(0);
            joints(index).Limit = struct();
            if ~isempty(limit)
                joints(index).Limit.effort = str2double(limit.getAttribute('effort'));
                joints(index).Limit.lower = str2double(limit.getAttribute('lower'));
                joints(index).Limit.upper = str2double(limit.getAttribute('upper'));
                joints(index).Limit.velocity = str2double(limit.getAttribute('velocity'));
            else
                joints(index).Limit.effort = 0;
                joints(index).Limit.lower = 0;
                joints(index).Limit.upper = 0;
                joints(index).Limit.velocity = 0;
            end
            index = index + 1;
        end
        
    end
    
    
    
    
    
    

end

