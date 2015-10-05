function PlotMatches( img1, Data1, img2, Data2)
% Function receives images ( img1, img2 ) and Data's.
% Data1( i, : ) - data of i-th FP in image 1 ( the same considertion applied to Data2 ).
% Data1( i, 1 ), Data1( i, 2 ) - row and column respectively of FP. Data1( i, 3 ) -
% information about length of square region ( can be of 2 Types, read below ).
% Data1( i, 4 ) - angle of square rotation ( main orientation ). Center of rotation is 
% [ Data1( i, 2 ), Data1( i, 1 ), 0 ] ( if we look at X, Y, Z coord. ). 
% TypeOfEdge - type of taking squares' length. If TypeOfEdge = 'sigma' -> square of size
% (12*Data1( i, 3 ) +1)x12*Data1( i, 3 ) +1). If TypeOfEdge = 'edge' -> ...
% Data1( i, 3 )xData1( i, 3 ).
% TypeOfFP - 'HL'/'Aff'. If 'HL' ( Harris-Laplace ) - will be ploted
% matched FP with according rectangles. 'Aff' - will be ploted corresp. FP,
% connected by line, without rectangles.
% Assumption - 1. angles are given in degrees.

%-------------------- Parameters -------------------%
LineWidth = 0.5;
EdgeColor = [ 200 20 20 ]/255;
LineColor = [ 30 100 100 ]/255;
FaceColor = 'none';
Linestyle = '-';

%------------------ Pre-processing -----------------%
% number of similar pairs
N = size( Data1, 1 );
[ m1, n1, tmp ] = size( img1 );
[ m2, n2, tmp ] = size( img2 );
rowY1 = Data1( :, 2 ); rowY1 = rowY1';
clmnX1 = Data1( :, 1 ); clmnX1 = clmnX1';
rowY2 = Data2( :, 2 ); rowY2 = rowY2';
clmnX2 = Data2( :, 1 ); clmnX2 = clmnX2';
% shifting column coordinates of second image, beause
% second image will be placed after first one
clmnX2 = clmnX2 + n1; 

%------------- Padding image if needed -------------%
if m1 > m2
    img2 = padarray( img2, [ m1 - m2 0 ], 'post' );
elseif m1 < m2
    img1 = padarray( img1, [ m2 - m1 0 ], 'post' );
end



%---------------- Finaly ploting results ------------------%
% show images
% figure;
hIm = montage( [ img1, img2 ] );
ax = gca;
axes(ax)
title('Corresponding FP between image1 and image2')

%--
hold on, plot( clmnX1, rowY1, 'g.' ), plot( clmnX2, rowY2, 'g.' );
%--
    
% plot all patches with lines
h = waitbar(0, 'Ploting/rotating patches:');
for i = 1:N
    line( [clmnX1( :, i ), clmnX2( :, i )], [rowY1( :, i ), rowY2( :, i )], 'Color', EdgeColor, 'LineWidth', LineWidth);
    waitbar( i/N )
end
close(h)

