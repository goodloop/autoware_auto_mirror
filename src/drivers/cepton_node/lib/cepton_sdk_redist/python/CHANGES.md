# Cepton SDK Python Release Notes

### Version 1.11 2019-01-25
* Fix `get_timestamp`.

### Version 1.10 2018-12-07
* Add `.py` extension to scripts for Windows support.

### Version 1.9 2018-11-02
* IMPORTANT: Combine `cepton_sdk.ImagePoints` and `cepton_sdk.Points`. Remove `cepton_sdk.ImagePoints`. All listeners now return `cepton_sdk.Points`.
* Change `cepton_sdk.wait` parameter `t_length` -> `duration`.

### Version 1.8 2018-10-08
* Update cepton_export.
* Make numpy properties immutable.

### Version 1.7 2018-10-02
* Fix timestamp jump in first frame.
* Allow only loading Python only components if shared library is not available.
* Rename some sample scripts, and make them globally available.

### Version 1.6 2018-09-04
* Update SDK.

### Version 1.5 2018-08-10
* Simplify api.
* Add listen/unlisten methods.
* Add listeners.
* Match version to SDK.

### Version 0.4 (beta) 2018-08-02
* Update SDK binaries.
* Update errors.

### Version 0.3 (beta) 2018-07-06
* Update SDK binaries.
* Add export points sample.

### Version 0.2 (beta) 2018-06-20
* Add cepton_util.

### Version 0.1 (beta) 2018-05-01
* Initial release.