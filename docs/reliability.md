# Reliability and EEPROM wear

[← Documentation index](README.md)

## Communication reliability

Messages from HeishaMon to the heatpump will occasionally be dropped, especially if multiple settings are changed within a short time. It's not feasible for HeishaMon to retry all dropped messages, so users should implement their own retry logic.

The suggested retry logic is as follows:
1. After changing a heatpump setting via HeishaMon, you should wait for HeishaMon to report that the setting has been changed.
2. If the setting isn't updated after 10 seconds, set it again.

This can be implemented on the HeishaMon itself using Rules if you wish, as in this example:
```
on StopExternalControl then
  if @External_Control != 0 then
    @SetExternalControl = 0;
    settimer(9,10);
  end
end

on timer=9 then
  if @External_Control != 0 then
    @SetExternalControl = 0;
  end
end
```


## EEPROM warning

As until today we don't know how the commands sent to the heatpump are processed in the heatpump itself. Most probably a lot of commands are written to EEPROM to be stored and available after a power failure, like setting the DHW temp. An EEPROM can facility a lot of writes but there is a limit. And we don't know the limit either. So make sure you don't overload the heatpump with too many commands. Every second is way too much. Just a few per hour, per settings, should probably be fine. Anyway, an heatpump is a slow heating(cooling) device so making changes that often is probably not even going to make any sense either.
