classdef Configurable < handle
    
    properties
        configParam;
    end
    
    methods (Abstract)
        CheckConfig(obj);
        SetDefaultValue(obj);
    end
    
    methods
        function obj = Configurable(cfgParam)
            SetDefaultValue(obj, cfgParam);
            CheckConfig(obj);
        end
    end
    
    methods (Static)
        function cfgParam = SetField(cfgParam, fldName, fldVal)
            if iscellstr(fldName)
                assert(iscell(fldVal), 'To set batch fields, the values must be in a cell array');
                assert(length(fldName) == length(fldVal), 'Number of field values must be equal to number of fiedl names');
                numFlds = length(fldName);
                if isempty(cfgParam)
                    cfgParam = struct(Vec(fldName), Vec(fldValue));
                else
                    b = isfield(cfgParam, fldName);
                    for iFld = 1:numFlds
                        if ~b(iFld)
                            cfgParam.(fldName{iFld}) = fldVal{iFld};
                        end
                    end
                end
            else
                assert(ischar(fldName), 'The parameter of fldName must be a string');
                if (isempty(cfgParam) || ~isfield(cfgParam, fldName))
                	cfgParam.(fldName) = fldVal;
                end
            end
        end
        
        function name = OneofName(oneofStruct)
            name = fieldnames(oneofStruct);
            assert(length(name) == 1, 'An oneof structure can have only one field');
            name = name{1};
        end
    end
end