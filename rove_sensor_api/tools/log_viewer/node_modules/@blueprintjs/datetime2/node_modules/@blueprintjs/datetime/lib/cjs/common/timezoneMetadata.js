"use strict";
/*
 * Copyright 2022 Palantir Technologies, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
Object.defineProperty(exports, "__esModule", { value: true });
exports.getTimezoneMetadata = void 0;
var timezoneItems_1 = require("./timezoneItems");
var timezoneNameUtils_1 = require("./timezoneNameUtils");
/**
 * Given a timezone IANA code and an optional date object, retrieve additional metadata like its common name, offset,
 * and abbreviation.
 */
function getTimezoneMetadata(timezoneIanaCode, date) {
    var timezone = timezoneItems_1.TIMEZONE_ITEMS.find(function (tz) { return tz.ianaCode === timezoneIanaCode; });
    if (timezone === undefined) {
        return undefined;
    }
    return (0, timezoneNameUtils_1.getTimezoneNames)(timezone, date);
}
exports.getTimezoneMetadata = getTimezoneMetadata;
//# sourceMappingURL=timezoneMetadata.js.map