/// <reference types="react" />
import type { DefaultPopoverTargetHTMLProps, Popover, PopoverProps } from "@blueprintjs/core";
/**
 * Reusable collection of props for components in this package which render a popover
 * and need to provide some degree of customization for that popover.
 */
export interface DatetimePopoverProps {
    /**
     * Props to spread to `Popover`.
     */
    popoverProps?: Partial<Omit<PopoverProps, "autoFocus" | "content" | "defaultIsOpen" | "disabled" | "enforceFocus" | "fill" | "renderTarget">>;
    /**
     * Optional ref for the popover component instance.
     * This is sometimes useful to reposition the popover.
     *
     * Note that this is defined as a specific kind of Popover which should be compatible with
     * most use cases, since it uses the default target props interface.
     */
    popoverRef?: React.RefObject<Popover<DefaultPopoverTargetHTMLProps>>;
}
