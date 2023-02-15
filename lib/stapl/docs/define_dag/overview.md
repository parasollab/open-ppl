# Overview

This document serves as an introduction to the functions provided in
`stapl/skeletons/operators/define_dag.hpp` for defining skeleton compositions.
This is an alternative to `inline_flows` but they can (and probably should) be
used together. Since inline flows lets you pass the output of any skeleton
anywhere, it is possible to create cycles and may be difficult to reason about
the high-level structure since every skeleton is "in scope" at every point. In
contrast, using the following functions, it is impossible to create cycles and
it encourages keeping only a few skeleton outputs "in scope." However, if it is
truly necessary to use a value in multiple places, it may be easier to use
inline flows and a descriptive name for the placeholder for those parts.

# Fundamental Functions

The implementation creates an AST which has 4 node types, aside from
regular skeletons. Each of these corresponds to a function that creates
one of these nodes from your skeletons. They are as follows.


## Serial / `ser(ts...)`

The first and simplest is `ser()`. In `ser(a,b)` the outputs of `a` become
the inputs of `b`. This function, like all the others, is variadic so we can
have `ser(a,b,c)` and so on.

This is the same as the default behavior of `skeletons::compose` except for the
fact that the skeletons we create can not only have multiple inputs, they can
also have *multiple outputs*. This brings us to the next function/node type.

## Parallel / `par(ts...)`

This function causes its arguments to be run in "parallel", independently of
each other. Let's look at an example, `par(a,b)`. The result is a skeleton which
has two outputs, the output of `a` and the output of `b`. They each get separate
inputs so the resulting skeleton has 2 inputs and 2 outputs.

Using the rules described above, we can see that
`ser(par(a,b), par(c,d)) = par(ser(a,c), ser(b,d))` as illustrated in this diagram.

![alt-text](parallel.png).

To explain it in words, the outputs of `par(a,b)` are `a` and `b`. And for `par(c,d)`
they each get their own input, so `c` consumes from `a` and `d` consumes from `b`.

Using a more terse notation, let `ab = ser(a,b)` and `(a,b) = par(a,b)`. So our
previous statement becomes `(a,b)(c,d) = (ac,bd)` which is component-wise
multiplication. This notation also motivates the creation of a left
distributive law `a(b,c) = (ab,ac)`, or in code `ser(a, par(b,c))`. In words, this means that if we serially
compose a single output skeleton `a` with one that takes multiple inputs, e.g.
`(b,c)`, we use it for all of `(b,c)`'s inputs.

![alt-text](split.png)


## Relabel / `rel<Is...>(s)`

This function lets you "relabel" the outputs of any given skeleton. For example,
`rel<1,0>(par(a,b))` flips the outputs to be as if you wrote `par(b,a)`. In addition
to permuting the outputs, you can drop or duplicate them, e.g. `rel<0>(par(a,b))`
performs `a` and `b` but only makes the output of `a` available to other skeletons.

So `ser(rel<0>(par(a,b)), par(c,d))` would result in the following graph. Note this
is the same as the first graph we saw except for the added `rel<0>()`.

![alt-text](relabel.png)


## Shift / `shift(ts...)`

One thing that is difficult to express with the 3 functions we've seen so far
is reusing the same input, over and over again. The only way we have to do that
so far is to duplicate it with `rel<>()` or make use of the left distributive law.
So if we wanted to create the following DAG

![alt-text](reuse.png)


we could do something like `ser(a, par(b, id), c)` where `id` is a special "identity"
skeleton. It serves as our identity element for serial composition and could be implemented
as something like `zip([](auto x) { return x; })`. This can clearly become quite involved
as we try and reuse `a` in more and more places.

This brings us to the final function, `shift()`. It is like `ser()` except each skeleton
gets the inputs to the first skeleton as well as all the outputs of all previous skeletons.
For example `shift(b,c)` yields

![alt-text](shift.png)

In this DAG, `c` gets the input to `b` as
well as the output of `b` - every possible input is passed to each skeleton.

Taking this a step further, `shift(b,c,d)` yields

![alt-text](shift2.png)

This isn't terribly
useful by itself, but we shall see how we can use it to construct any DAG with the
help of a secondary function, `take`, which is built upon `ser()`, `rel<>()` and `id`.


# Secondary Functions

## `take<Is...>(s)` - Relabelling inputs instead of outputs

`take<Is...>(s)` is for inputs as `rel<Is...>(s)` is to outputs. It allows you to
filter, duplicate, or permute the inputs to a skeleton just as we can for its outputs with `rel<>()`.

If we change the previous skeleton to use `take` like `shift(b,c,take<1,2>(d))`, we get

![alt-text](shift_take.png)

For those that are interested, `take` can be implemented as `ser(rel<Is...>(id), s)`.

Now we can prove that with these functions, we can easily define any DAG. Looking at the
nodes (skeletons) in the compose DAG in topological order, their inputs are subsets of
the previous nodes. With `shift()` each skeleton gets all of the them and with `take<>()`
we can pick the correct subsets.

# Converting to `inline_flows`

In the same header file, we get `stapl::skeletons::to_skeleton(S const& s)`
which converts a skeleton defined with these functions into a `compose` skeleton
with the corresponding `inline_flow`.

However, now you don't need to worry about calling
`DECLARE_INLINE_PLACEHOLDERS` and `DECLARE_INLINE_INPUT_PLACEHOLDERS` and
having warnings about unused variables if you have too many or an error if you
have too few. Furthermore, it is impossible to create a cycle using these functions.

*Note*: if you call `to_skeleton()` on a skeleton with multiple outputs, e.g. `par(a,b)`,
the last skeleton will be used as the output. This is because when it comes time to actually
run the skeleton, there is not yet support for multiple output. And since we're converting to
`inline_flows` the last skeleton listed in the flow is assumed to be the output.
