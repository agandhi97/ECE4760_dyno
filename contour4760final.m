
   %%%%%%%
   %%% Creates contour map for efficiency using rpm and torque measurements. 
   %%% inputs: 
        %%filename: character array, i.e 'text.txt'
        %%skipl: number of header lines to skip
   %%%%%%%
   
    function contour4760final(filename, skipl)

    %%% process data
    
    filecontent = fileread(filename); %reads file name
    
    newcontent = regexprep(filecontent, {'\r', '\n\n+', '\n'}, {'', '\n', '\r\n'}); %gets rid of spaces before/after each line
    ff = fopen('processdata.txt', 'w'); %creates new text file to store processed data; do this for reference and testing
    fwrite(ff, newcontent); %write parsed data to text file
    fclose(ff); %close file 
    
    fid = fopen('processdata.txt', 'r'); %open processed data 
    %initialize vectors for rpm, torque, efficiency
    R = []; 
    T = [];
    E = [];
    
    %skip unwanted lines; i.e Putty header
    if(skipl > 0)
        for i = 1:skipl
            fgetl(fid); 
        end
    end
    
    
    while(~feof(fid)) %checks if every line has been read
           line = fgetl(fid);  %reads first line
           line = line(1, 2:end); %stores first line as character array
    
           if(length(line) > 1) %checks if line has data or is garbage
               lineVect = textscan(line, '%f'); %scan line for float values and store in cells
               %append values to vectors
               R = [R; lineVect{1}(1)];
               T = [T; lineVect{1}(2)];
               E = [E; lineVect{1}(3)]; 
           end 
    end
    fclose(fid); %close file
    
    %get length of vectors
    rL = length(R)
    tL = length(T)
    eL = length(E)
    
    %check if lengths of rpm, torque, efficiency arrays are the same
    if( rL ~= tL && rL ~= eL && eL ~= tL)
       ss = min([rL tL eL]); %find min length
       %readjust array size to match minimum
       R = R(1, 1:ss); 
       T = T(1, 1:ss); 
       E = E(1, 1:ss); 
    end

    %%%%create contour map
    
    %create linear space for rpm and torque, interpolates over 100 points
    x0 = min(R); x1 = max(R); nx = 100;
    y0 = min(T); y1 = max(T); ny = 100;
    x = linspace(x0, x1, nx);
    y = linspace(y0, y1, ny);
    [X, Y] = meshgrid(x, y); %creates mesh grid of x, y coordinates
    Z = griddata(R, T, E, X, Y); %interpolates Z with respect to x and y
    
    contourf(X, Y, Z) %create map
    colorbar 
    xlabel('RPM'); 
    ylabel('Torque');
    title('Efficiency Contour Map'); 

    end 
