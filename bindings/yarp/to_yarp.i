%pythonbegin %{
import scenario_bindings
from typing import Union
%}

%extend scenario::base::World {
  %pythoncode %{
    def to_yarp() -> Union["World", scenario_bindings.World]:
        pass
  %}
}

%extend scenario::base::Model {
  %pythoncode %{
    def to_yarp() -> Union["Model", scenario_bindings.Model]:
        pass
  %}
}
