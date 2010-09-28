function plot_Isomap(R,pname,fname);

     dims=1:6

%http://www.mathworks.com/support/solutions/data/1-5V3AV0.html?solution=1-5V3AV0
%http://www.mathworks.com/access/helpdesk/help/techdoc/index.html?/access/helpdesk/help/techdoc/ref/axes_props.html
%clear

     figure;close
     y2=myslope(myslope(R));
     dims2=((length(dims)-length(y2))+1):1:length(dims);
     [trash,maxarraypos]=max(ismember(y2,max(y2)));
     maxarraypos=dims2(maxarraypos);
     clear trash;


     hold on
     plot(dims, R, 'b-o');
     plot(maxarraypos,R(maxarraypos), 'r*', 'MarkerSize',20);
     %ylim([0 0.24]);
     ylabel('Residual Variance');
     xlabel('Dimensionality');
     title(pname);

     ha=gca;
     set(gca, 'XTick',[1 2 3 4 5 6]);
     uistack(ha,'bottom');

     haPos=get(ha,'position');
     ha2=axes('position', [(haPos(1:2)+.4),.275,.275,] );

     hold on
     plot(dims2, y2, 'k-o');
     plot(maxarraypos,max(y2), 'r*','MarkerSize',20);
     set(gca, 'FontSize',8);
     set(gca, 'XTick',[1 2 3 4 5 6]);
     xlim([1 6]);
     ylabel('Elbow');
     xlabel('Dimensionality');

     print('-djpeg', fname)

end
