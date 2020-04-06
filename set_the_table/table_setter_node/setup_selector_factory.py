import table_setter_basic
class setup_selector_factory:
    def setup_selector_factory_method(self,setup_type):
        # ask and edit
        return table_setter_basic.TableSetter()
        if setup_type == "set_table":
            object_ = table_setter_basic.TableSetter()
        return object_