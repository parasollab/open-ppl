function [maxarraypos] = plot_Isomap(R,pname,fname);

    Siz= size(R);
     dims=[1:Siz(2)];
ticks = [1:1:Siz(2)];
%http://www.mathworks.com/support/solutions/data/1-5V3AV0.html?solution=1-5V3AV0
%http://www.mathworks.com/access/helpdesk/help/techdoc/index.html?/access/helpdesk/help/techdoc/ref/axes_props.html
%clear

     y2=myslope(myslope(R));
     dims2=((length(dims)-length(y2))+1):1:length(dims);
     [trash,maxarraypos]=max(ismember(y2,max(y2)));
     maxarraypos=dims2(maxarraypos);
     clear trash;
end
