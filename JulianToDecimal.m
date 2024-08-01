function dy= JulianToDecimal(jy)
    JD_1970 = 2440587.5; % Julian date for 1970-01-01 00:00:00 UTC
    %sdateTime = datetime(jy, 'ConvertFrom', 'juliandate');
    %Y = year(dateTime);
    %if (mod(Y,4) ==0)
        %SECONDS_IN_YEAR = 366 * 24 * 3600;
    %else 
        SECONDS_IN_YEAR = 365.2428005 * 24 * 3600;
        %365.2428005 
    %end
    %SECONDS_IN_YEAR_common = 365* 24 * 3600
    %SECONDS_IN_DAY = 24 * 3600

    % Convert Julian date to Unix timestamp (seconds since 1970-01-01)
    unixTime = (jy - JD_1970) * 86400;
    % Calculate the year

    % dateTime = datetime(jy, 'ConvertFrom', 'juliandate');
    Y = 1970 + unixTime / SECONDS_IN_YEAR;
    %Calculate the decimal part of the year
    currentYear = floor(Y);
    % %currentYear = year(dateTime);
    % number_leap_before = mod((currentYear - 1972),4)
    startOfYear = (currentYear - 1970) * SECONDS_IN_YEAR;%_common + number_leap_before*SECONDS_IN_DAY;
    startOfNextYear = (currentYear + 1 - 1970) * SECONDS_IN_YEAR;
    %Fraction of the year that has passed
    fractionOfYear = (unixTime - startOfYear) / (startOfNextYear - startOfYear);
    % fractionOfYear = (unixTime - startOfYear) / SECONDS_IN_YEAR;
    % % Combine integer year and fraction
    %format long
    dy = currentYear + fractionOfYear;
end

% function dy = JulianToDecimal(jy)
% dateTime = datetime(jy, 'ConvertFrom', 'juliandate')
% Y = year(dateTime)
% dayOfMonth = [31 28 31 30 31 30 31 31 30 31 30 31]
% if (mod(Y,4) ==0)
%     dayOfMonth(2) = 29
% end
% 
% M = month(dateTime)
% D = day(dateTime)
% B = dateshift(dateTime,'start','year');
% E = B + calyears(1);
% Y = Y + (dateTime-B) ./ (E-B);
% dy = Y
% [y,m,d] = ymd(dateTime)
% dy = y+
% end

