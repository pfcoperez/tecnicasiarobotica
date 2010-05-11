function dibujaGrafo(MA)
    %MA = Matriz de adyacencia
    ta = size(MA);
    [bn, coords] = bucky;
    d = floor(60/ta(1));
    gplot(MA, coords(1:(d-1):60,2:3));
    hold on;
    posiciones = coords(1:(d-1):60,2:3);
    xtemp=posiciones(:,1);
    ytemp=posiciones(:,2);
    for k=0:(ta(1)-1)
        text(xtemp(k+1)+0.05,ytemp(k+1)+0.05,num2str(k))
        plot(xtemp(k+1),ytemp(k+1),'or');
    end
    %line([.25 0 0 .25 NaN 3.75 4 4 3.75],[0 0 4 4 NaN 0 0 4 4]);
    %axis off;
    hold off;
end
