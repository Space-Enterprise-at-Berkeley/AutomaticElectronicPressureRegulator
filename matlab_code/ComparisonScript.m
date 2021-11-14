compare_table = readtable("Comparison.xlsx");
plot(compare_table.Angle1, compare_table.Cv1, "g");
hold on
plot(compare_table.Angle2, str2double(compare_table.Cv2), "b");
hold on
plot(compare_table.Angle3, compare_table.Cv3, "r");
hold on
plot(compare_table.Angle4, compare_table.Cv4, "o");
hold on