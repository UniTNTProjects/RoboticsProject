
myfile4 = fopen('quaterror.txt','w');

rotation = [-1.0, 0.0, 0.0; 
    0.0, -1.0, 0.0;
    0.0, 0.0, 1.0];

cord = [-0.15; 0.15; 0.15];

dim_x = 20;
dim_y = 20;
dim_z = 10;

for p = 0:dim_z
    for i = 0:dim_x
        for j = 0:dim_y
            position = cord + [0.05 * i; -0.05 * j; 0.05 * p];

            inverse_kinematics_res = ur5Inverse(position, rotation);
            min_error = 5000;
            for k = 1:8
                th = inverse_kinematics_res(k, :);
                [cord1, rot] = ur5Direct(th);
                
                q1 = quaternion(rotm2quat(rot));
                q2 = quaternion(rotm2quat(rotation));            
                error = dist(q1,q2);
                
                if error< min_error
                    min_error = error;
                end   
            end
            fprintf(myfile4,'%e ',min_error);

        end
        fprintf(myfile4, '\n');
    end
     fprintf(myfile4, '%f\n', cord(3) + 0.05 * p);
end
fclose(myfile4);
