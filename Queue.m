classdef Queue < handle
    properties (Access = private)
        elements
        nextInsert
        nextRemove
    end

    properties (Dependent = true)
        NumElements
    end

    methods
        function obj = Queue
            obj.elements = cell(1, 50);
            obj.nextInsert = 1;
            obj.nextRemove = 1;
        end
        
        function push(obj, el)
            if obj.nextInsert == length( obj.elements )
                obj.elements = [ obj.elements, cell( 1, length( obj.elements ) ) ];
            end
            obj.elements{obj.nextInsert} = el;
            obj.nextInsert = obj.nextInsert + 1;
        end
        
        function pop_front(obj)
            if obj.isEmpty()
                error( 'Queue is empty' );
            end
            obj.elements{ obj.nextRemove } = [];
            obj.nextRemove = obj.nextRemove + 1;
            % Trim "elements"
            if obj.nextRemove > ( length( obj.elements ) / 2 )
                ntrim = fix( length( obj.elements ) / 2 );
                obj.elements = obj.elements( (ntrim+1):end );
                obj.nextInsert = obj.nextInsert - ntrim;
                obj.nextRemove = obj.nextRemove - ntrim;
            end
        end
        
        function n = next(obj)
            if obj.isEmpty
                n = [];
            else
                obj.pop_front();
                n = obj.elements{ obj.nextRemove };
            end
        end
        
        function f = front(obj)
            if obj.isEmpty
                f = [];
            else
                f = obj.elements{ obj.nextRemove };
            end
        end
        
        function b = back(obj)
            if obj.isEmpty
                b = [];
            else
                b = obj.elements{ obj.nextInsert - 1 };
            end
        end
        
        
        function clear(obj)
            obj.elements = cell(1, 50);
            obj.nextInsert = 1;
            obj.nextRemove = 1;
        end
        
        
        function tf = isEmpty(obj)
            tf = ( obj.nextRemove >= obj.nextInsert );
        end
        
        function n = get.NumElements(obj)
            n = obj.nextInsert - obj.nextRemove;
        end
        
    end
end