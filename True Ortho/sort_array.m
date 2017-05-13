function test_sort = sort_array(test_zip,pixel_Angle,test_x,test_y) %function
    global occlude_area;
    for i = 1:length(test_zip);
        test_z(i,1) = pixel_Angle(test_zip(i,1),test_zip(i,2));
    end
    test_array = [round(test_x') round(test_y') test_z];
    for i = 1:length(test_array)
        if test_array(i,3)<max(test_array(1:i-1,3))
            test_array(i,3) = -10000;
        end
    occlude_area(test_array(i,2),test_array(i,1)) = test_array(i,3);
    end
    %occlude_area(sub2ind(size(occlude_area),test_array(:,2),test_array(:,1))) = test_array(:,3);