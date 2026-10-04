// Adapted from Emscripten 6.0.3 libembind.js. Copyright 2012 The Emscripten
// Authors. Distributed under the MIT license; see embind_boundary.LICENSE.
// This adapter supports synchronous wasm32 bindings with Emscripten JS EH only.
#if WASM_EXCEPTIONS || ASYNCIFY || JSPI || MEMORY64 || DISABLE_EXCEPTION_CATCHING
throw new Error('PhysicsEngine Embind boundary requires synchronous wasm32 JS exception handling');
#endif

addToLibrary({
  $physicsSizingGetters: {},
  $physicsPrepareArguments__deps: ['$physicsSizingGetters'],
  $physicsPrepareArguments: function(name, self, args) {
    // A foreign JS exception from an array accessor inside emscripten::val
    // bypasses C++ RAII in JS EH. Snapshot these bounded public array inputs
    // before entering native code; check every shape before reading any entry.
    let arrays, count, maxwell = false, elastic = false, electrostatic = false;
    if (name === 'PeriodicScalarTransport.setState' || name === 'PeriodicScalarTransport.setVelocities') {
      const config = physicsSizingGetters['PeriodicScalarTransport.getConfig'].call(self);
      count = config.columns * config.rows;
      if (count > 262144) throw new RangeError('Scalar cell cap exceeded');
      arrays = args;
    } else if (name === 'PeriodicElectrostaticGrid.solve') {
      const config = physicsSizingGetters['PeriodicElectrostaticGrid.getConfig'].call(self);
      count = config.columns * config.rows;
      if (count > 262144) throw new RangeError('Electrostatic cell cap exceeded');
      arrays = [args[0]];
      electrostatic = true;
    } else if (name === 'PeriodicMacGrid.setVelocities') {
      const config = physicsSizingGetters['PeriodicMacGrid.getConfig'].call(self);
      count = config.columns * config.rows;
      if (count > 262144) throw new RangeError('MAC cell cap exceeded');
      arrays = args;
    } else if (name === 'WaveMembrane.setState') {
      count = physicsSizingGetters['WaveMembrane.getCellCount'].call(self);
      arrays = args;
    } else if (name === 'ElasticWaveGrid.setState') {
      const config = physicsSizingGetters['ElasticWaveGrid.getConfig'].call(self), state = args[0];
      count = config.columns * config.rows;
      if (count > 262144) throw new RangeError('Elastic wave cell cap exceeded');
      arrays = [state.vx, state.vy, state.sigmaXX, state.sigmaYY, state.sigmaXY];
      elastic = true;
    } else if (name === 'MaxwellGrid.setState') {
      const config = physicsSizingGetters['MaxwellGrid.getConfig'].call(self), state = args[0];
      count = config.columns * config.rows;
      if (count > 262144) throw new RangeError('Maxwell cell cap exceeded');
      arrays = [state.ez, state.hx, state.hy];
      maxwell = true;
    } else return args;
    if (!Number.isSafeInteger(count) || count <= 0) throw new RangeError(name + ' invalid native cell count');
    const lengths = [];
    for (const array of arrays) {
      if (!Array.isArray(array)) throw new TypeError(name + ' requires JavaScript arrays');
      lengths.push(array.length);
    }
    if (lengths.some(length => length !== count))
      throw new RangeError(name + ' array length does not match the grid');
    const copies = arrays.map(array => {
      const copy = new Array(count);
      for (let i = 0; i < count; ++i) {
        if (!Object.prototype.hasOwnProperty.call(array, i))
          throw new TypeError(name + ' arrays must be dense');
        const value = array[i];
        if (typeof value !== 'number' || !Number.isFinite(value))
          throw new TypeError(name + ' entries must be finite numbers');
        // Define own entries: polluted Array.prototype setters must not run.
        Object.defineProperty(copy, i, {value, writable: true, enumerable: true, configurable: true});
      }
      return copy;
    });
    if (electrostatic) {
      if (args.length === 1) return copies;
      // Capture options before receiver wiring too: an options getter may
      // delete the owner or make a reentrant bound call.
      const options = {};
      for (const key of ['absoluteGaussTolerance', 'relativeGaussTolerance', 'maximumIterations', 'maximumCellVisits']) {
        const value = args[1][key];
        if (typeof value !== 'number' || !Number.isFinite(value))
          throw new TypeError(name + ' options require finite numbers');
        Object.defineProperty(options, key, {value, enumerable: true});
      }
      return [copies[0], options];
    }
    if (elastic) return [{vx: copies[0], vy: copies[1], sigmaXX: copies[2], sigmaYY: copies[3], sigmaXY: copies[4]}];
    return maxwell ? [{ez: copies[0], hx: copies[1], hy: copies[2]}] : copies;
  },

  $physicsEmbindCall__deps: ['$stackSave', '$stackRestore', '$getExceptionMessage',
    '__cxa_begin_catch', '__cxa_end_catch'],
  $physicsEmbindCall: function(body) {
    const saved = stackSave();
    try {
      return body();
    } catch (error) {
      if (!(error instanceof CppException)) throw error;
      // begin/end catch balance both the native uncaught counter and the one
      // reference acquired by __cxa_throw. Never rethrow a released CppException.
      ___cxa_begin_catch(error.excPtr);
      let owned;
      try {
        const [type, message] = getExceptionMessage(error);
        owned = new Error(message || type);
        owned.name = type || 'Error';
      } finally {
        ___cxa_end_catch();
      }
      throw owned;
    } finally {
      stackRestore(saved);
    }
  },

  $craftInvokerFunction__deps: ['$physicsEmbindCall', '$runDestructors',
    '$createNamedFunction', '$throwBindingError', '$getRequiredArgCount', '$physicsPrepareArguments', '$physicsSizingGetters'],
  $craftInvokerFunction: function(humanName, argTypes, classType, cppInvokerFunc, cppTargetFunc, isAsync) {
    if (isAsync) throwBindingError('PhysicsEngine does not support async bindings');
    const count = argTypes.length - 2;
    const minimum = getRequiredArgCount(argTypes);
    const method = argTypes[1] !== null && classType !== null;
    const invoker = createNamedFunction(humanName, function(...args) {
      return physicsEmbindCall(() => {
        if (args.length < minimum || args.length > count)
          throwBindingError(`${humanName} called with ${args.length} arguments; expected ${minimum}..${count}`);
        args = physicsPrepareArguments(humanName, this, args);
        // Per-call storage makes conversion reentrant. Every converter receives
        // a destructor stack, including those normally using the fast path.
        const destructors = [];
        try {
          const wired = [cppTargetFunc];
          if (method) wired.push(argTypes[1].toWireType(destructors, this));
          for (let i = 0; i < count; ++i)
            wired.push(argTypes[i + 2].toWireType(destructors, args[i]));
          const result = cppInvokerFunc(...wired);
          return argTypes[0].isVoid ? undefined : argTypes[0].fromWireType(result);
        } finally {
          runDestructors(destructors);
        }
      });
    });
    // Capture native sizing observers before exposing mutable JS prototypes.
    // A user-shadowed getConfig/getCellCount cannot enlarge snapshot work.
    if (humanName === 'PeriodicMacGrid.getConfig' || humanName === 'WaveMembrane.getCellCount' ||
        humanName === 'MaxwellGrid.getConfig' || humanName === 'PeriodicScalarTransport.getConfig' ||
        humanName === 'ElasticWaveGrid.getConfig' || humanName === 'PeriodicElectrostaticGrid.getConfig')
      physicsSizingGetters[humanName] = invoker;
    return invoker;
  },

  _embind_finalize_value_object__deps: ['$structRegistrations', '$runDestructors',
    '$readPointer', '$whenDependentTypesAreResolved'],
  _embind_finalize_value_object: function(structType) {
    const reg = structRegistrations[structType];
    delete structRegistrations[structType];
    const records = reg.fields;
    const types = records.map(f => f.getterReturnType).concat(records.map(f => f.setterArgumentType));
    whenDependentTypesAreResolved([structType], types, fieldTypes => {
      const fields = {};
      records.forEach((field, i) => {
        const readType = fieldTypes[i], writeType = fieldTypes[i + records.length];
        fields[field.fieldName] = {
          read: ptr => readType.fromWireType(field.getter(field.getterContext, ptr)),
          write: (ptr, value) => {
            const destructors = [];
            try { field.setter(field.setterContext, ptr, writeType.toWireType(destructors, value)); }
            finally { runDestructors(destructors); }
          },
          optional: readType.optional,
        };
      });
      return [{
        name: reg.name,
        fromWireType: ptr => {
          try {
            const result = {};
            for (const name in fields) result[name] = fields[name].read(ptr);
            return result;
          } finally { reg.rawDestructor(ptr); }
        },
        toWireType: (destructors, object) => {
          for (const name in fields)
            if (!(name in object) && !fields[name].optional) throw new TypeError(`Missing field: "${name}"`);
          const ptr = reg.rawConstructor();
          let transferred = false;
          try {
            for (const name in fields) fields[name].write(ptr, object[name]);
            if (destructors !== null) destructors.push(reg.rawDestructor, ptr);
            transferred = true;
            return ptr;
          } finally { if (!transferred) reg.rawDestructor(ptr); }
        },
        readValueFromPointer: readPointer,
        destructorFunction: reg.rawDestructor,
      }];
    });
  },

  _embind_register_class_property__deps: ['$AsciiToString', '$embind__requireFunction',
    '$runDestructors', '$throwBindingError', '$throwUnboundTypeError',
    '$whenDependentTypesAreResolved', '$validateThis', '$physicsEmbindCall'],
  _embind_register_class_property: function(classType, fieldName, getterReturnType,
      getterSignature, getter, getterContext, setterArgumentType, setterSignature, setter, setterContext) {
    fieldName = AsciiToString(fieldName);
    getter = embind__requireFunction(getterSignature, getter);
    whenDependentTypesAreResolved([], [classType], types => {
      const type = types[0], name = `${type.name}.${fieldName}`;
      const unbound = () => throwUnboundTypeError(`Cannot access ${name} due to unbound types`, [getterReturnType, setterArgumentType]);
      Object.defineProperty(type.registeredClass.instancePrototype, fieldName,
        {get: unbound, set: unbound, enumerable: true, configurable: true});
      whenDependentTypesAreResolved([], setter ? [getterReturnType, setterArgumentType] : [getterReturnType], fieldTypes => {
        const descriptor = {
          get: function() { return physicsEmbindCall(() => {
            const ptr = validateThis(this, type, name + ' getter');
            return fieldTypes[0].fromWireType(getter(getterContext, ptr));
          }); },
          enumerable: true,
        };
        if (setter) {
          setter = embind__requireFunction(setterSignature, setter);
          descriptor.set = function(value) { return physicsEmbindCall(() => {
            const destructors = [];
            try {
              const ptr = validateThis(this, type, name + ' setter');
              setter(setterContext, ptr, fieldTypes[1].toWireType(destructors, value));
            } finally { runDestructors(destructors); }
          }); };
        } else descriptor.set = () => throwBindingError(name + ' is a read-only property');
        Object.defineProperty(type.registeredClass.instancePrototype, fieldName, descriptor);
        return [];
      });
      return [];
    });
  },

  _embind_register_class_class_property__deps: ['$AsciiToString', '$embind__requireFunction',
    '$runDestructors', '$throwBindingError', '$throwUnboundTypeError',
    '$whenDependentTypesAreResolved', '$physicsEmbindCall'],
  _embind_register_class_class_property: function(rawClassType, fieldName, rawFieldType, rawFieldPtr,
      getterSignature, getter, setterSignature, setter) {
    fieldName = AsciiToString(fieldName);
    getter = embind__requireFunction(getterSignature, getter);
    whenDependentTypesAreResolved([], [rawClassType], types => {
      const type = types[0], name = `${type.name}.${fieldName}`;
      const unbound = () => throwUnboundTypeError(`Cannot access ${name} due to unbound types`, [rawFieldType]);
      Object.defineProperty(type.registeredClass.constructor, fieldName,
        {get: unbound, set: unbound, enumerable: true, configurable: true});
      whenDependentTypesAreResolved([], [rawFieldType], fieldTypes => {
        const fieldType = fieldTypes[0];
        const descriptor = {
          get: () => physicsEmbindCall(() => fieldType.fromWireType(getter(rawFieldPtr))),
          enumerable: true,
        };
        if (setter) {
          setter = embind__requireFunction(setterSignature, setter);
          descriptor.set = value => physicsEmbindCall(() => {
            const destructors = [];
            try { setter(rawFieldPtr, fieldType.toWireType(destructors, value)); }
            finally { runDestructors(destructors); }
          });
        } else descriptor.set = () => throwBindingError(name + ' is a read-only property');
        Object.defineProperty(type.registeredClass.constructor, fieldName, descriptor);
        return [];
      });
      return [];
    });
  },
});
