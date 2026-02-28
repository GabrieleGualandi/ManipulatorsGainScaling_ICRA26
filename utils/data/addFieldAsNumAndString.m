function [numericOut, stringOut] = addFieldAsNumAndString(numericIn, stringIn, dataFieldName, data, nDigitString)
% addFieldAsNumAndString   Updates structures with both numeric and string versions of data.

    numericOut = numericIn;
    numericOut.(dataFieldName) = data;

    stringOut = stringIn;
    stringOut.(dataFieldName) = numToMantissaNotation(data, nDigitString);    
end