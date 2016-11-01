function [idxY,idxX] = YinX2(x, y)
	[xx, yy] = meshgrid(x, y);
	[row, col] = find(xx-yy==0);
	idxY = row;
	idxX = col;
end