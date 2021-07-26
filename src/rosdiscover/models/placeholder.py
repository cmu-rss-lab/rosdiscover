from ..interpreter import model


@model('PLACEHOLDER', 'PLACEHOLDER')
def placeholder(c):
    # c.pub("/diagnostics", "diagnostic_msgs/DiagnosticArray")
    c.mark_placeholder()
